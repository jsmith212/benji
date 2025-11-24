# 5. Version and build all apps as a single entity

Date: 2025-11-23

## Status

Accepted

## Context

Benji is built as a monorepo containing libraries, microcontroller apps (Zephyr), and Linux user-space apps. The compute platform image build uses the Yocto Project (via kas) to compose a base image plus our layer that includes these apps. Additionally, the RT application may leverage some of the same libraries, and will be bundled with the compute image for deployment.

Especially in the early days of the project we want:

- High velocity development, minimal administrative overhead.
- Developers to change libraries/apps and quickly validate via the image build.
- A clearly traceable release build, but we are not yet expecting extremely fine-grained versioning of each individual component.

This allows for two ways to version apps in the image:

1. **Per-app pinning**: each app recipe pins its `SRCREV` (or fetch source) so the image includes exactly known commits of each app. This allows for independent versioning of each app, but requires updating multiple recipes for each release, and ultimately causes additional release management overhead.
2. **Monorepo snapshot + externalsrc**: use `externalsrc` in the recipes so the Yocto build picks up from the working tree at a known monorepo commit, treating the monorepo as the version unit. This reduces overhead, but means all apps share the same version (the monorepo commit) with no way to override this.

Both allow for reproducible builds, it is only the granularity that is impacted. It should be noted that this is not a permanent decision; as the project matures we may want to move to per-app pinning for more flexibility and traceability.

## Decision

We will adopt option (2) — **monorepo snapshot + externalsrc** — for the current project phase.

- For release builds: we will check out the monorepo at a specific commit or tag and ensure that tag is embedded in the image metadata.
- Our Yocto layer’s app recipes will `inherit externalsrc` (or equivalent) to pull directly from the monorepo working tree.
- We will disable per-app `SRCREV` pinning during this phase.
- We will document this in our release workflow and ensure developers understand: “All apps in the image come from this monorepo commit.”

## Consequences

Currently, the trade-offs listed below are acceptable given our priorities. As the project evolves and stabilizes, we may revisit this decision.

### Pros

- Reduced administrative burden: no need to update multiple recipes’ `SRCREV` values each release.
- Faster iteration: changes in libraries/apps can be built and integrated quickly.
- Clear version unit: the monorepo commit is the version for the full image.

### Cons

- We lose independent versioning of individual apps (e.g., we cannot easily keep app A at version X while moving app B to version Y, unless we branch the monorepo).
- For reproducibility, we rely on a clean working tree and correct checkout; mistakes (uncommitted changes) may lead to unknown image contents.
- The `externalsrc` approach may bypass some of Yocto’s canonical fetching behaviour and therefore may demand extra discipline (e.g., ensuring source archives, consistent path, build history).
- If we scale to many apps and variant images, we may hit limitations of this model and incur a migration cost later.
