# Third-party notices

## CMU eVTOL Battery Dataset — active battery reference

The battery data are attributed to **Alexander Bills et al., eVTOL Battery
Dataset, CMU, version 3**, DOI `10.1184/R1/14226830.v3`. The pinned dataset
metadata records the data license as **CC BY 4.0**. Source URLs, file identities,
checksums and transformation provenance are retained with the products below;
the original laboratory data must not be presented as this project's drone
measurements.

| Derived product | Use and changes to the source |
|---|---|
| `profiles/battery/cmu_runtime/` | Active runtime reference. Four measured RPT stages from the complete VAH01 lifetime CSV; derived relative capacity, effective DC 1-second resistance estimates and SOC-indexed equivalent OCV curves. The 4S2P pack, initial SOC and pack scaling are simulation assumptions. Curves are not measured equilibrium OCV. |

Attribution and detailed source records:

- [Runtime derivation and source references](docs/cmu-runtime-profiles.md),
  [catalog](profiles/battery/cmu_runtime/catalog.json), and
  [provenance](profiles/battery/cmu_runtime/provenance.json).
- [Current service guide](docs/cmu-service-guide.md). Runtime telemetry is a
  load-driven simulation derived from the reference, not a replay of measured
  pack voltage/current and not evidence of real-vehicle AI accuracy.

The initial-normal six-profile bundle and its replay tooling are removed from
the retained tree. Downloaded CMU originals and historical validation outputs
are not retained locally after the approved purge. The runtime catalog, curve
CSVs, provenance and `tools/battery_profiles/cmu_aging.py` remain; regeneration
and the real-source publication test require reacquiring the pinned inputs
described in [runtime derivation](docs/cmu-runtime-profiles.md#재생성).

NASA battery products and tooling are no longer part of the active execution
path. The retired code archive, downloaded originals and historical experiment
outputs are also outside the retained tree; see
[NASA retirement](docs/nasa-retirement.md). Remaining historical documents do
not imply that their raw evidence is still available. NASA inputs are not a
dependency of the CMU service and are not silently combined with CMU inputs.

## PX4-derived material

`models/custom_x500/model.sdf` is derived from PX4-Autopilot v1.17.0
(`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`). Stock motor physics is retained.
The PX4 integration follows its module lifecycle template. The original license
applies to that material:

```text
BSD 3-Clause License

Copyright (c) 2012 - 2023, PX4 Development Team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
