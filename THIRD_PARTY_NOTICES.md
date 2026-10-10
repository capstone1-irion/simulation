# Third-party notices

## PX4 Gazebo x500 model settings

`models/capstone_x500/model.sdf` is a V2-owned experiment definition derived from the motor settings of PX4/PX4-gazebo-models, revision `b6127f4ec20de867e215fb5f78ae88b80f371909`. It includes the unmodified `x500_base` dependency and redirects the four motor command subscriptions through the V2 relay. The source checkout is not overwritten. The source license notice is retained below.

BSD 3-Clause License

Copyright (c) 2022, PX4 Autopilot for Drones
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
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

## Battery-derived data

The source attribution, CC BY 4.0 metadata and original extraction limitations for the CMU eVTOL Battery Dataset version 3 are retained in `profiles/battery/provenance.json` and described in `docs/배터리모델.md`. V2 reorganizes existing derived values; it does not claim a new raw-data derivation, independent cells, or validation by the dataset authors. This notice does not assign an open license to the previously supplied motor specification image.

## Karpathy guidelines

`docs/karpathy-guidelines/SKILL.md` preserves the selected text from multica-ai/andrej-karpathy-skills revision `2c606141936f1eeef17fa3043a72095b4765b9c2`. Its frontmatter declares MIT. A separate LICENSE file was not present at the inspected pinned source, so no replacement license text was invented.

## Dependencies

The PX4 checkout, Gazebo system packages and Python dependencies retain their respective original licenses. They are not relicensed by this project's notices. The project has not selected an overall license for all newly authored V2 code.
