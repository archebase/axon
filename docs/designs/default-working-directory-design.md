# Default Working Directory Policy

**Date:** 2026-03-02
**Status:** Design

## Overview

This document defines the default working directory policy for Axon applications.

The policy standardizes where each application reads and writes runtime data by introducing two user-selected modes:

- **Product mode**: base directory is `/axon`
- **Dev mode**: base directory is `/tmp/axon`

Mode selection is a user decision. This document intentionally does not prescribe an implementation mechanism (no required environment variable, flag, or detection rule).

## Policy

All application runtime paths are derived from one selected base directory.

| Mode | Base Directory |
|------|----------------|
| Product | `/axon` |
| Dev | `/tmp/axon` |

Per-application default directories:

| Application | Relative Directory | Product Path | Dev Path |
|------------|--------------------|--------------|----------|
| `axon_recorder` | `recording` | `/axon/recording` | `/tmp/axon/recording` |
| `axon_config` | `config` | `/axon/config` | `/tmp/axon/config` |
| `axon_transfer` | `transfer` | `/axon/transfer` | `/tmp/axon/transfer` |

## Required Consistency

1. A deployment/session must use exactly one mode at a time.
2. `axon_recorder`, `axon_config`, and `axon_transfer` must use paths derived from the same base directory in that deployment/session.
3. Defaults and examples in design docs and config templates should follow this policy.

## Rationale

- Product mode keeps persistent runtime data under `/axon`.
- Dev mode avoids elevated permission requirements and allows safe local iteration under `/tmp/axon`.
- A shared base directory keeps inter-application contracts stable (recordings, config cache, transfer state).

## Scope

This policy applies to default runtime directories only.

It does not define:

- Mode selection mechanism
- CLI/API surface changes
- Migration scripts
