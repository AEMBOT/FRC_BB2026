# FRC_BB2026
AEMBOT robot code for BunnyBots 2026 (Cone Zone). Based on [AEMTemplate](https://github.com/AEMBOT/AEMTemplate),
with [AEMLib](https://github.com/AEMBOT/AEMLib) as a subtree at `src/main/java/com/aembot/lib`.

## What's here
No mechanisms yet. The field and game are simmed so drive, vision and scoring can be worked on first.

* `constants/field/FieldBB2026` - field geometry, tower stations, arena, human player corners, cones,
  starting lines and the 12 tag apriltag layout. `Field2026` is still the FRC field, pick one with
  `RobotRuntimeConstants.FIELD`.
* `simulation/arena` - maplesim arena with walls, the 8 tower stations and the arena as one solid
  block. Places the 42 starting cones, spawns human player cones and bunnies. Stacks and scoring are
  in `SimulatedTowerStations` and `ConeStack`.
* `simulation/gamepieces` - cones on the carpet and in flight.
* `simulation/intake/SimulatedConeIntakeState` - intake collider and toss. A light toss with the
  bumper against a station seats the cone, anything else ends up on the carpet. Its geometry and
  toss numbers come from `ConeIntakeConfiguration` in `config/robots/ProductionConeConfig`.
* `simulation/vision` - `ConeCameraSim` renders the field from the cone limelight's position,
  `ConeStackPipeline` is a java port of the Limelight's OpenCV pipeline and runs on every frame, and
  `SimulatedConeLimelight` fills the `limelight-cones` table with what it found (tv, tx, ty, ta,
  llpython). Any station in view is a target, bare or stacked, so the robot can line up on an
  empty one. The ConeCam stream shows the pipeline's boxes like the real Limelight would.
* `state/SimulatedRobotStateYearly` - installs the arena, runs the cone camera on its own thread
  and ties it all together for logging.
* `commands/SimulationCommandFactory` - sim only bindings on the secondary controller.

Apriltag vision is the template's Limelight 4 stack running off photonvision sim with the BunnyBots layout.

## Sim controls (secondary controller)
* A (hold) - run the intake, drive into a cone to grab it
* Y - light toss onto the station in front of the robot
* B - drop the held cone
* X - human player throws in the bunny
* LB / RB - blue / red human player drops a cone
* Back - reset the field

## Simulating
Run **WPILib: Simulate Robot Code**. The cone camera streams as `ConeCam` on `http://localhost:1181`.

AdvantageScope assets (field, cone game pieces, a robot with a matching Front Camera) come from
[Field_BB2026](https://github.com/AEMBOT/Field_BB2026) `tools/install_advantagescope_assets.sh|.bat`.
Robot pose is `SimulatedRobotState/RobotPose2d`. Field side state is under `SimulatedArenaState`:
cones are `Cones/RED|BLUE|WHITE`, stacks are `Stacks/StationN` and `Stacks/Center`, score is `Score/Red|Blue`.

`CONE_LIMELIGHT_SOURCE` in `SimulatedRobotStateYearly` picks who fills the `limelight-cones` table.
`PIPELINE` (default) runs the java port of the OpenCV pipeline on every rendered frame, `GROUND_TRUTH`
publishes what the renderer knows is there with no vision noise, `EXTERNAL` leaves the table alone
so `tools/limelight/limelight_sim.py` from Field_BB2026 can run the actual python file against the
ConeCam stream.

## Real robot
Field layouts for the coprocessors are in Field_BB2026, `2026-bunnybots-conezone.json` for
photonvision and `.fmap` for limelight. The cone pipeline and its bench are there too.

## Things to check at the event
* Starting lines are guessed at 10' from center, the drawing doesn't dimension them
* Each nested cone raises a stack 3" (`SimulatedTowerStations.NESTED_CONE_PITCH_METERS`)
* Camera position, intake geometry and toss strength are all placeholders
