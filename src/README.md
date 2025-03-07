# FRC Team 9140: 2025 Robot Code

We are currently developing the robot for the 2025 FRC game *Reefscape*. While we are still in the design phase, we are actively coding and testing the components we are certain about using simulation.

## Libraries

Our project includes a set of utility classes and libraries designed to simplify common tasks and improve reusability across robots. These classes are contained within the `lib` folder.

### MazeRunner

`MazeRunner` is a wrapper class around *ChoreoLib*, developed to simplify the process of creating and managing autonomous sequences. We found that the default *ChoreoLib* implementation is overly complex for our needs, so `MazeRunner` provides a simpler API containing only the key features which we believed were essential.

Key benefits:
- Simplified API: Avoids the need for using `AutoFactory` and other redundant classes from *ChoreoLib*.
- Easier debugging: Automatically logs the trajectory and robot's expected position, which makes discrepancies easier to identify while testing autonomous sequences.