# PRD: Interactive Billiards Simulator "Geometry in Motion"

## 1. Overview
Develop a "sandbox" type web application where the user can experiment with billiard balls on a finite plane. The system should be visually appealing and allow for the intuitive validation of Geometric Analysis theorems (up to the Circumference).

## 2. User Requirements (UX/UI)
- **"8 Ball Pool" Interface:** The user should be able to drag the mouse to aim and release to shoot (controlling force and direction).

- **Ball Spawning:** A button to add a cue ball (spin ball) and multiple colored balls (targets) in random or fixed positions.

- **Theory Mode:** A panel of switches to activate/deactivate the display of concepts without cluttering the screen:

- View Normal Lines and Angles.

- View Symmetric Point P*.

- See Congruent Triangles (SAS).

- See Points of Tangency (Circles).

## 3. Technical Specifications (Based on Sources)
- **Professional Table:** Rectangle with a 2:1 ratio [Berrío et al.].

- **Balls (Circle):** Modeled as perfect circles. The ball-ball collision is detected by the distance between centers (r1 + r2), ensuring tangency [Guide Text, Chapter VI].

- **Kinematic Motion:** Use of the Harrison model for precise displacements to "target positions" calculated using reflection vectors: v' = v - 2(v·n)n.

## 4. Educational Visualization
- Upon impact with the rail, show the arc of the incident and reflected angles (α = β).

- Draw the perpendicular bisector of the segment formed between the ball and its mirror point P* to demonstrate isometry.