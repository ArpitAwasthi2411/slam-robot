# Let an AI assistant use Pathik (MCP)

`scripts/mcp_server.py` is a small [Model Context Protocol](https://modelcontextprotocol.io) server.
It runs on your laptop, talks to the robot's HTTP API, and gives an assistant like Claude 13 tools:
`robot_status`, `list_places`, `go_to`, `go_to_thing`, `command`, `find_thing`, `explain`, `cancel`,
`go_home`, `charge`, `confirm_ok`, `schedule`, `mission_stats`. No packages to install.

## Claude Desktop
Settings → Developer → Edit config, add:
```json
{"mcpServers": {"pathik": {
  "command": "python3",
  "args": ["/home/you/slam-robot/scripts/mcp_server.py"],
  "env": {"ROBOT_URL": "http://slam-robot:8080"}
}}}
```
`ROBOT_URL`: `http://10.42.0.1:8080` on the robot's Wi-Fi, `http://slam-robot:8080` over Tailscale,
`http://169.254.1.2:8080` on the ethernet cable, `http://localhost:8080` for the simulator.

## Claude Code
```bash
claude mcp add pathik -e ROBOT_URL=http://slam-robot:8080 -- python3 scripts/mcp_server.py
```

## Try
- "Where is Pathik and how much battery does it have?"
- "Send it to the room with the 3D printer, and tell me when it gets there." (it polls `robot_status`)
- "It stopped. Why?"
- "Every weekday at 10:00, patrol Lab 2 then Lab 3 and come back."
- "Summarise today's missions for my report."

## Safety
The assistant can only do what the app can: the e-stop, the RC override, the battery rules and
the robot's own obstacle and body checks still apply. It cannot release an e-stop.
Test against the simulator first: `python3 tools/dashboard_sim.py` and `ROBOT_URL=http://localhost:8080`.


## Login

The robot needs a login since v2.5. Make an account in the Pathik app (an admin approves it; staff can send the robot, operator can also edit places and schedules) and give it to the server:
`ROBOT_USER=... ROBOT_PASSWORD=...` next to `ROBOT_URL` in the MCP config.
