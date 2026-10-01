
        occupied = {
            (int(m.state.position[0]), int(m.state.position[1]))
            for m in self.robot_managers
            if m is not manager and m.state.status != RobotStatus.OFFLINE
        }

        # A service bay is also reserved by an AMR that has already selected
        # it as its post-task destination. In P2P mode there is intentionally
        # no fleet-wide reservation table, so the simulator must not hand the
        # same fixed bay to two robots during local post-task scheduling.
        peer_service_targets = {
            (int(m.target_cell[0]), int(m.target_cell[1]))
            for m in self.robot_managers
            if (
                m is not manager
                and m.state.status != RobotStatus.OFFLINE
                and m.target_cell is not None
                and m.post_task_mode in ("CHARGER", "STAGING")
            )
        }

        candidates = []
        for cell in cells:
            cell = (int(cell[0]), int(cell[1]))
            if cell in occupied or cell in peer_service_targets:
                continue
            # A manager may carry a stale reservation for a service cell from
            # an earlier route. That self-claim must not make the resource look
            # unavailable to the same manager.
            claimer = manager.reservation_table.get_claimer(
                cell, float(self.tick_count + 1)
            )
            if claimer is not None and claimer != manager.state.robot_id:
                continue
            distance = (
                abs(cell[0] - int(manager.state.position[0]))
                + abs(cell[1] - int(manager.state.position[1]))
            )
            candidates.append((distance, cell))

        candidates.sort()
        return candidates[0][1] if candidates else None

    def _schedule_post_task_destination(self, manager: LocalTaskManager, mode: str) -> bool:
        """Send a taskless AMR to a fixed charger or staging bay.

        This is fleet housekeeping, not order generation, so it continues even
        when Auto Mode is OFF. A delivery cell is released immediately when a
        task completes and is never used as a parking destination.
        """
        if manager.current_task is not None or manager.state.status == RobotStatus.OFFLINE:
            return False

        if mode not in ("CHARGER", "STAGING"):
            raise ValueError(f"Unsupported post-task mode: {mode}")

        cells = self.charger_cells if mode == "CHARGER" else self.staging_cells
        goal = self._find_free_service_cell(manager, cells)
        manager.post_task_mode = mode

        if goal is None:
            manager.target_cell = None
            manager.state.planned_path = []
            manager.state.status = RobotStatus.WAITING
            manager.waiting_on = f"{mode}_RESOURCE"
            manager.checkpoint_reached = True
            self.event_log.log_conflict(
                manager.state.robot_id,
                "SYSTEM",
                f"{mode}_RESOURCE",
                "WAIT_RESOURCE",
                self.tick_count,
            )
            return False

        manager.target_cell = goal
        manager.state.planned_path = []
        manager.state.status = RobotStatus.MOVING
        manager.wait_time = 0.0
        manager.waiting_on = None
        manager.checkpoint_reached = True
        if not manager.cbs_mode:
            manager._replan()
        self.event_log.log_conflict(
            manager.state.robot_id,
            "SYSTEM",