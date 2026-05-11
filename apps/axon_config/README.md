# Axon Config Application

Robot initialization, configuration collection, and device registration tool.

## Commands

```bash
axon-config init
axon-config scan
axon-config enable
axon-config disable
axon-config status
axon-config register --factory "Factory Shanghai" --robot-type SynGloves --keystone-url http://keystone:8080
```

The dispatcher also exposes the registration shortcut:

```bash
axon register --factory "Factory Shanghai" --robot-type SynGloves --keystone-url http://keystone:8080
```

## Keystone Device Registration

`register` sends a JSON request to Keystone:

```http
POST /api/v1/devices/register
Content-Type: application/json
```

```json
{
  "factory": "Factory Shanghai",
  "robot_type": "SynGloves"
}
```

On success, Keystone returns `201 Created` and `axon-config` prints the JSON response to stdout.
On failure, `axon-config` prints the HTTP status and response body to stderr and exits non-zero.

The Keystone base URL can be passed with `--keystone-url` or the `AXON_KEYSTONE_URL` environment
variable:

```bash
AXON_KEYSTONE_URL=http://keystone:8080 axon register \
  --factory "Factory Shanghai" \
  --robot-type SynGloves
```
