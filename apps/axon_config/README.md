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
axon-config refresh
```

The dispatcher also exposes shortcuts for registration and config refresh:

```bash
axon register --factory "Factory Shanghai" --robot-type SynGloves --keystone-url http://keystone:8080
axon refresh
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

On success, Keystone returns `201 Created`, `axon-config` persists the registration state to
`/etc/axon/device.json`, and prints the JSON response to stdout. On failure, `axon-config`
prints the HTTP status and response body to stderr and exits non-zero.

After a successful registration, `register` downloads runtime config templates from Keystone:

```text
GET <keystone>/configs/<factory>/<robot_type>/recorder.yaml
GET <keystone>/configs/<factory>/<robot_type>/transfer.yaml
```

The downloaded templates are saved under `/etc/axon/templates/` as `recorder.yaml.tpl`
and `transfer.yaml.tpl`. `axon-config` renders them with the local registration context
and writes `/etc/axon/recorder.yaml` and `/etc/axon/transfer.yaml`, replacing the
package-installed defaults only after both templates download and render successfully.
Use `--config-dir <dir>` to write elsewhere, or `--skip-config-download` to skip the
template/config step while still saving `device.json`. If Keystone returns `404` for a
config path, the command prints a warning, leaves existing templates/configs unchanged,
and keeps the registration successful. Writing to `/etc/axon` requires sufficient
permissions.

Templates use `{{ name }}` placeholders. Available values include Keystone response
fields and local derived fields:

```yaml
device_id: "{{ device_id }}"
factory_id: "{{ factory_id }}"
robot_type_id: "{{ robot_type_id }}"
robot_id: "{{ robot_id }}"
keystone_url: "{{ keystone_url }}"
recorder_rpc_url: "{{ recorder_rpc_url }}"
transfer_ws_url: "{{ transfer_ws_url }}"
```

Nested JSON values can also be addressed with dotted names, for example
`{{ endpoints.transfer_ws_url }}`. Unknown placeholders fail the command and leave
existing rendered configs unchanged.

## Refreshing Configs

Use `refresh` after a device has already been registered and `/etc/axon/device.json`
exists:

```bash
axon-config refresh
```

`refresh` does not call Keystone's registration API and does not change `device.json`.
It reads `factory`, `robot_type`, and `keystone_url` from `device.json`, downloads the
current recorder/transfer templates, and regenerates the rendered YAML files with the
saved robot identity. Use `--keystone-url` to override the saved Keystone URL, or
`--config-dir <dir>` to refresh a non-default config directory.

The Keystone base URL can be passed with `--keystone-url` or the `AXON_KEYSTONE_URL` environment
variable:

```bash
AXON_KEYSTONE_URL=http://keystone:8080 axon register \
  --factory "Factory Shanghai" \
  --robot-type SynGloves
```
