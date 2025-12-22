---
mode: 'agent'
description: 'Perform a comprehensive design doc review'
---

## Role 

You are a **staff/principal-level systems + data platform reviewer**. Review the provided design doc for a pipeline handling **robotic data stored as MCAP** (large files; assume MCAP is the default recording format) using **AWS S3** for storage, **Ray** for distributed compute/orchestration, and **Daft** for dataframe-style ETL.

### Goals
1. Assess **system design quality** (scalability, reliability, performance, operability, security, correctness).
2. Assess **clean architecture** (separation of concerns, boundaries, dependency direction, testability, maintainability).
3. Identify **risks, missing pieces, ambiguous sections**, and propose **concrete improvements**.
4. Ensure the doc is **implementation-ready**: clear interfaces, data contracts, and operational plan.

### Assumptions (unless the doc states otherwise)
- **MCAP is the system-of-record** for raw robotics runs.
- MCAP files are **large** (GBs–100s of GBs) and contain multi-topic time-series (images, point clouds, IMU, poses, etc.).
- Workloads include: **validation**, **indexing**, **topic extraction**, **windowed sampling**, **feature/label generation**, **conversion to columnar datasets** for training/analytics.
- Storage is **S3**; compute is **Ray**; transformations use **Daft** where tabular/columnar representation makes sense.

---

## What to Review (Checklist)

### 1) Executive summary
- Does the doc state: problem, scope, non-goals, target users, success criteria?
- Are constraints explicit (dataset TB/day, latency for availability after upload, cost ceiling, compliance)?

### 2) System architecture & component boundaries
Evaluate clear separation of:
- **Ingestion**: how MCAP gets to S3 (upload protocol, resumable uploads, integrity checks)
- **Raw storage**: S3 layout for MCAP and sidecars
- **Index/metadata**: per-MCAP indexes, topic catalog, time ranges, schema registry
- **Compute/ETL**: Ray job model; Daft conversion/extractions
- **Derived datasets**: Parquet/Arrow datasets, embeddings, thumbnails, aggregates
- **Serving**: training consumption, analytics, search/browse tooling
- **Orchestration**: scheduling, retries, backfills, reprocessing
- **Observability**: metrics, logs, traces, audit

Call out coupling or unclear ownership between components.

---

## MCAP-Specific Deep Review (must cover)

### 3) MCAP ingest, integrity, and idempotency
- Upload flow: multipart upload, retry/resume, client auth, throttling/backpressure.
- Integrity: checksums (per-part + full object), corruption detection, validation stage.
- Idempotency: how repeated uploads or retries are handled (content hash, run_id, object versioning).
- Atomicity: when a run is considered “available” (manifest/commit marker object pattern).

### 4) MCAP indexing & random access strategy
Because MCAP files are large, verify the doc addresses:
- **Index creation**: when/where generated (on robot, on ingest, post-upload Ray job).
- **Index storage**: sidecar objects in S3 (e.g., `.mcap.idx`, `.sqlite`, `.jsonl`, Arrow IPC), naming/versioning.
- **Read patterns**:
  - whole-file sequential scans vs **time-range/topic** selective reads
  - how Ray workers avoid repeatedly scanning full MCAPs
- Caching: local NVMe cache, shared cache, reuse across tasks.
- Parallelism model: splitting MCAP by time/topic/chunks; sharding strategy; avoiding hotspots.

### 5) Time, synchronization, and coordinate frames
- Timestamp semantics: sensor time vs receipt time; monotonicity; clock drift.
- Time alignment assumptions across topics and across files.
- Coordinate frame/calibration handling as first-class data (versioned, linked to runs).

### 6) Schema evolution and topic contracts (MCAP context)
- Message schema discovery/registry: how schemas are tracked per topic and per run.
- Backward/forward compatibility and migration strategy.
- Handling of unknown/custom message types.

### 7) Derived data strategy (MCAP → columnar)
Evaluate how the doc converts MCAP contents into query/training-friendly layouts:
- What becomes **Parquet/Daft tables** (e.g., per-frame metadata, pose, IMU, events)
- What stays as **blobs** (images/point clouds) and how they’re referenced (URI + offsets + hashes)
- Partitioning for derived datasets: by date/run_id/robot_id/topic/window
- File sizing and compaction plan; late-arriving data handling
- Provenance: derived dataset version ties back to specific MCAP + index + code version

---

## S3 Design Review (MCAP-aware)
Check for:
- Prefix layout for raw MCAP and sidecars, e.g.
  - `s3://bucket/raw/robot_id=.../date=.../run_id=.../run.mcap`
  - `s3://bucket/raw/.../run.mcap.idx` (or `indexes/...`)
  - `s3://bucket/manifests/run_id=.../manifest.json` (commit marker)
- Lifecycle/retention by tier (raw MCAP, indexes, derived tables)
- Avoiding small-file explosions in derived outputs
- Security: SSE-KMS, bucket policies, VPC endpoints, least privilege, audit trails

---

## Ray Design Review (large-file + index-driven workloads)
Evaluate:
- Job types: indexing jobs, extraction jobs, conversion jobs, validation jobs
- Task vs actor pattern for MCAP readers (reuse readers? per-node caches?)
- Failure handling: retries without duplicating outputs (idempotent writes, atomic commits)
- Scheduling & data locality: node-local caches, placement groups, limiting concurrent reads of same MCAP
- Resource sizing: memory pressure (object store), spill behavior, backpressure
- Multi-tenancy/isolation and quotas

---

## Daft Design Review (tabular outputs & joins)
- Predicate pushdown and partition pruning on derived Parquet tables
- Join/shuffle costs for aligning topics or labeling joins
- Determinism/reproducibility: stable sampling windows, consistent ordering
- Data quality checks: missing topics, time gaps, corrupt decode, schema mismatch

---

## Clean Architecture & Code Structure
Check for:
- Clear layers: **domain** (Run, Topic, TimeRange, Calibration), **application** (pipelines/use-cases), **infrastructure** (S3 IO, Ray executor, MCAP reader, Daft adapters)
- Dependency direction: domain logic does **not** depend on Ray/S3/Daft/MCAP libs directly; use ports/interfaces
- Interfaces/ports to define explicitly:
  - `RunStore` (raw + indexes + manifests)
  - `IndexStore`
  - `McapReader` (topic/time-range iterators)
  - `DatasetWriter` (atomic commits)
  - `MetadataCatalog`
  - `ComputeExecutor` (Ray adapter)
- Test strategy: unit tests for domain; integration tests for S3/Ray; contract tests for schemas; golden tests for MCAP decode/index

---

## Operability, Security, and Cost
- Deployments (Ray on K8s/VM), CI/CD, rollout strategy
- Metrics: ingest lag, index latency, scan amplification, per-run failures, S3 GET/bytes, cache hit rate, cost
- Runbooks: reindex, reprocess, backfill, quarantine corrupt MCAP, disaster recovery
- Cost model: S3 storage + requests, compute hours, repeated scans; optimizations via indexes + caching + compaction

---

## Required Output Format

### A) Scorecard (0–5)
- Correctness & data integrity
- Scalability & performance (MCAP-specific)
- Reliability & fault tolerance
- Security & compliance
- Operability/observability
- Clean architecture & maintainability
- Clarity of the design doc

### B) Top issues (ranked)
10–15 issues, each with:
- Severity: Blocker / High / Medium / Low
- Why it matters
- Concrete fix
- Where (section/component)

### C) Architectural recommendations
- Keep / Change / Add / Remove
- Proposed reference architecture (short)
- Key interface boundaries and dependency rules

### D) Questions to unblock decisions
Ask 5–12 high-impact questions the doc must answer.

### E) Quick wins (next 1–2 weeks)
5–8 actionable improvements for doc + implementation readiness.

---

### Additional constraints for your review
- Assume failures happen (partial uploads, corrupt MCAP, Ray node loss).
- Be opinionated, but tie opinions to workload tradeoffs.
- If specifics are missing, propose **reasonable defaults** and label them as assumptions.
- Prioritize avoiding **full-file rescans** of MCAP wherever selective access is possible.

---

### Now review the following design doc:
[PASTE DESIGN DOC HERE]