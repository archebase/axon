# Axon Transfer Application

`axon-transfer` is the standalone upload daemon for recording artifacts. It scans the local
recording directory, persists upload state in SQLite, uploads MCAP files to S3-compatible storage,
and reports operator-facing status over the fleet WebSocket.

## Completion Contract

The scanner supports two completion modes:

- `require_json_sidecar: false`: MCAP-only mode. A recording is upload-ready only when
  `<task_id>.mcap` and `<task_id>.mcap.done` both exist. The marker mtime must be at least as new as
  the MCAP mtime, so a file that changes after the marker is skipped.
- `require_json_sidecar: true`: legacy mode. A recording is upload-ready only when `<task_id>.mcap`
  and `<task_id>.json` both exist. The JSON sidecar is still uploaded after the MCAP object.

`completion_marker_suffix` defaults to `.done`. `min_ready_age_ms` can add a quiet period for both
the MCAP and completion sentinel when deployments need extra protection against filesystem latency.

## Upload State

Upload state is stored in `uploader.state_db_path` and survives process restarts. Records include:

- local MCAP path and optional JSON sidecar path
- S3 object key, factory id, device id, and task id
- MCAP file size and SHA-256 checksum
- status, retry count, last error, next retry time, and completion time

The active upload statuses are:

- `pending`: queued but not yet claimed by a worker
- `active`: currently uploading
- `retry-wait`: waiting for exponential backoff before retry
- `uploaded_wait_ack`: S3 upload completed and local cleanup is waiting for fleet ACK
- `completed`: ACK/cleanup finished, or cleanup is disabled
- `failed`: retry budget exhausted or a permanent local/S3 error occurred

Older `uploading` records are migrated to `active` on startup.

## Status Reporting

The WebSocket status payload exposes counts for pending, active, retry-wait, waiting-ACK, completed,
and failed uploads. It also includes pending bytes, aggregate bytes uploaded, current throughput,
destination summary (`bucket`, `endpoint_url`, `region`), and per-upload diagnostic fields such as
`checksum_sha256`, `file_size_bytes`, `s3_key`, `retry_count`, `last_error`, and `next_retry_at`.

Credentials are intentionally omitted from status payloads and logs.

## Configuration

Example scanner configuration:

```yaml
scanner:
  data_dir: "/tmp/axon/recording"
  require_json_sidecar: false
  completion_marker_suffix: ".done"
  min_ready_age_ms: 0
```

Example uploader configuration:

```yaml
uploader:
  state_db_path: "/tmp/axon/transfer/transfer_state.db"
  delete_after_upload: true
  failed_uploads_dir: "/tmp/axon/transfer/failed_uploads/"
  num_workers: 2
  s3:
    endpoint_url: "https://s3.amazonaws.com"
    bucket: "axon-recordings"
    region: "cn-northwest-1"
```

`AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, `AXON_S3_BUCKET`, `AXON_S3_ENDPOINT`,
`AXON_TRANSFER_DATA_DIR`, and `AXON_TRANSFER_WS_URL` can override selected YAML fields.

## Migration Notes

For MCAP-first deployments, recorder finalization writes `<task_id>.mcap.done` after the MCAP writer
is closed and metadata handling is complete. Switch transfer configs to `require_json_sidecar:
false` to consume that marker. Legacy MCAP+JSON deployments can keep `require_json_sidecar: true`
without changing object layout.
