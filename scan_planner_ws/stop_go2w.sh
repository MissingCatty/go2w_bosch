#!/bin/bash
set +e
systemctl --user stop go2-scan-planner-dry.service >/dev/null 2>&1
docker stop -t 5 go2_scan_planner_dry >/dev/null 2>&1
systemctl --user reset-failed go2-scan-planner-dry.service >/dev/null 2>&1
echo "SCAN-Planner 已停止；LIO-SAM 保持运行"
