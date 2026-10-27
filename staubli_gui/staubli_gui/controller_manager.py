"""Gestión de los controladores del robot (joint_trajectory_controller,
forward_velocity_controller y joint_state_broadcaster).

Reglas del proyecto:
  - Los modos TRAYECTORIA y MANUAL (control FRAME) son EXCLUYENTES: nunca
    pueden estar activos forward_velocity_controller y
    joint_trajectory_controller a la vez.
  - El joint_state_broadcaster debe estar siempre activo.
  - Los cambios de controlador se hacen por servicio /controller_manager/switch_controller
    con strictness STRICT (si algo sale mal NO se cambia nada) y se verifican después
    con /controller_manager/list_controllers.
  - STRICT no es idempotente: si se pide activar un controlador ya activo la llamada
    falla. Por eso antes del switch se filtra con el estado más reciente qué
    controladores hay que cambiar de verdad.

Este módulo es independiente de Qt: solo usa ROS 2 (rclpy), para poder probarlo
sin servidor de ROS.
"""

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController

ACTIVE_STATE = "active"
INACTIVE_STATE = "inactive"
UNCONFIGURED_STATE = "unconfigured"
FINALIZED_STATE = "finalized"

JOINT_TRAJECTORY_CONTROLLER = "joint_trajectory_controller"
FORWARD_VELOCITY_CONTROLLER = "forward_velocity_controller"
JOINT_STATE_BROADCASTER = "joint_state_broadcaster"


class ControllerManager:
    """Cliente de /controller_manager para consultar y cambiar controladores."""

    def __init__(self, node, switch_timeout_s=5.0):
        self._node = node
        self._switch_timeout_s = max(int(switch_timeout_s), 1)
        self._states = {}
        self._list_client = node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_client = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    # ── Estado ───────────────────────────────────────────────────────────

    def refresh(self, done_callback=None):
        """Consulta /controller_manager/list_controllers de forma asíncrona.

        done_callback(states, ok, error): states es un dict nombre -> estado.
        Si /controller_manager no está disponible, ok es False.
        """
        if not self._list_client.service_is_ready():
            if done_callback:
                self._node.get_logger().debug(
                    "servicio /controller_manager/list_controllers no disponible"
                )
                done_callback({}, False, "controller_manager no disponible")
            return False
        future = self._list_client.call_async(ListControllers.Request())
        future.add_done_callback(lambda f: self._on_list_done(f, done_callback))
        return True

    def _on_list_done(self, future, done_callback):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            if done_callback:
                done_callback({}, False, f"error consultando controladores: {exc}")
            return
        self._states = {c.name: c.state for c in response.controller}
        if done_callback:
            done_callback(dict(self._states), True, None)

    # ── Cambios ──────────────────────────────────────────────────────────

    def switch(self, *, activate=(), deactivate=(), done_callback=None):
        """Cambio ATÓMICO y verificado de controladores.

        - Refresca el estado real para filtrar las peticiones ya satisfechas
          (STRICT falla si se pide activar algo ya activo).
        - Hace /controller_manager/switch_controller con STRICT en una sola
          llamada: o cambia todo o no cambia nada.
        - Verifica después con list_controllers que `activate` quedan activos
          y `deactivate` inactivos.

        activate/deactivate: listas de NOMBRES de controlador.
        done_callback(ok, states, message).
        """
        if not self._switch_client.service_is_ready():
            if done_callback:
                try:
                    self._node.get_logger().warning(
                        "servicio /controller_manager/switch_controller no disponible"
                    )
                except Exception:
                    print("[ControllerManager] servicio /controller_manager/switch_controller no disponible")
                done_callback(False, dict(self._states),
                              "controller_manager no disponible")
            return False
        self.refresh(
            done_callback=lambda st, ok, err: self._after_refresh_before_switch(
                st, ok, err, list(activate), list(deactivate), done_callback))
        return True

    def _after_refresh_before_switch(self, states, ok, error, activate,
                                     deactivate, done_callback):
        """Tras conocer el estado real, decide qué cambiar de verdad."""
        if not ok:
            if done_callback:
                done_callback(
                    False, dict(self._states),
                    f"no se pudo verificar el estado antes del cambio: {error}",
                )
            return
        activar = [n for n in activate if states.get(n) != ACTIVE_STATE]
        desactivar = [n for n in deactivate if states.get(n) == ACTIVE_STATE]
        if not activar and not desactivar:
            if done_callback:
                done_callback(
                    True, dict(states),
                    "los controladores ya estaban en el estado deseado",
                )
            return
        request = SwitchController.Request()
        request.activate_controllers = activar
        request.deactivate_controllers = desactivar
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout = Duration(sec=self._switch_timeout_s)
        try:
            future = self._switch_client.call_async(request)
        except Exception as exc:  # noqa: BLE001
            if done_callback:
                done_callback(False, dict(self._states),
                              f"no se pudo pedir el cambio: {exc}")
            return
        future.add_done_callback(
            lambda f: self._on_switch_done(f, activate, deactivate, done_callback)
        )

    def _on_switch_done(self, future, activate, deactivate, done_callback):
        """Comprueba la respuesta del servicio y verifica el estado final."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            if done_callback:
                done_callback(False, dict(self._states),
                              f"switch_controller falló en la llamada: {exc}")
            return
        if not response.ok:
            if done_callback:
                done_callback(False, dict(self._states), response.message)
            return

        def _confirm(states, refresh_ok, refresh_error):
            if not refresh_ok:
                if done_callback:
                    done_callback(False, dict(self._states),
                                  f"verificación posterior fallida: {refresh_error}")
                return
            faltan_activos = [n for n in activate if states.get(n) != ACTIVE_STATE]
            quedan_activos = [n for n in deactivate if states.get(n) == ACTIVE_STATE]
            if faltan_activos or quedan_activos:
                detalle = " ; ".join(
                    [f"no activo: {n}" for n in faltan_activos]
                    + [f"aún activo: {n}" for n in quedan_activos]
                )
                if done_callback:
                    done_callback(False, dict(states),
                                  f"cambio de controladores incompleto: {detalle}")
                return
            if done_callback:
                done_callback(True, dict(states), response.message)

        self.refresh(done_callback=_confirm)


# ── Derivación de modo y plan de switch (funciones puras para probar) ────


def derive_mode(states):
    """A partir del estado de los controladores deduce el modo actual.

    Devoluciones:
      "trayectoria"  → JTC activo y FVC inactivo.
      "manual"       → FVC activo y JTC inactivo.
      "seguro"       → ninguno de los dos activo (o controller_manager muy
                       inicializado).
      "conflicto"    → JTC y FVC activos a la vez (estado prohibido).
      "desconocido"  → no se tiene información fiable (ej. sin respuesta).
    """
    jtc = states.get(JOINT_TRAJECTORY_CONTROLLER)
    fvc = states.get(FORWARD_VELOCITY_CONTROLLER)
    if jtc == ACTIVE_STATE and fvc == ACTIVE_STATE:
        return "conflicto"
    if jtc == ACTIVE_STATE:
        return "trayectoria"
    if fvc == ACTIVE_STATE:
        return "manual"
    if jtc in (INACTIVE_STATE, UNCONFIGURED_STATE, FINALIZED_STATE) or \
            fvc in (INACTIVE_STATE, UNCONFIGURED_STATE, FINALIZED_STATE):
        return "seguro"
    return "desconocido"


def switch_plan(mode):
    """Controladores a activar/desactivar para alcanzar `mode`.

    Garantiza exclusividad: cada plan activa su controlador y desactiva el otro.
    joint_state_broadcaster siempre va en activate (debe quedar activo).
    """
    if mode == "trayectoria":
        return {
            "activate": (JOINT_TRAJECTORY_CONTROLLER, JOINT_STATE_BROADCASTER),
            "deactivate": (FORWARD_VELOCITY_CONTROLLER,),
        }
    if mode == "manual":
        return {
            "activate": (FORWARD_VELOCITY_CONTROLLER, JOINT_STATE_BROADCASTER),
            "deactivate": (JOINT_TRAJECTORY_CONTROLLER,),
        }
    if mode == "seguro":
        return {
            "activate": (JOINT_STATE_BROADCASTER,),
            "deactivate": (JOINT_TRAJECTORY_CONTROLLER, FORWARD_VELOCITY_CONTROLLER),
        }
    return {"activate": (), "deactivate": ()}