<a href="https://team230.org/">
    <img src="https://github.com/Gaelhawks-Team-230/FRC-team230-2024/assets/51682341/0a425013-5d06-494b-b9c6-021ecc4e6b84" alt="Gaelhawks logo" title="Aimeos" align="right" height="200" />
</a>

# Gaelhawks 2024 Monorepo

![Relative date](https://img.shields.io/date/1709874000?style=for-the-badge&label=Waterbury%20District%20Event&labelColor=black&color=f8651d)

## Table of Contents

* [Getting Started](#getting-started)
* [Usage](#usage)
* [Contributing](#contributing)
  * [Important Notes](#important-notes)
  * [Commit Code](#commit-code)
  * [Merge Code with Pull Requests](#merge-code-with-pull-requests)
* [Examples](#examples)

## Getting Started

<details>
    <summary>Installation Instructions</summary>

### Developer Install

1. Installing WPILib
    * Follow the [official WPILib installation instructions](https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)
2. *New students only* Installing Git and GitHub for repository management
    * Install [Git](https://git-scm.com/downloads) for your OS and use default install settings
    * Add Git to the path - search `Edit Environment Variable`, choose `Path`, then select `System Variables.` Next, add the Git installed folder path and save changes. Run `git –version` in the terminal to verify install
    * Create a GitHub Account and ask Luke to add you to the 2024 Student Programming Team
    * Create a local development folder, we recommend `C:\work` and clone the repository with `git clone <URL>`
3. Launch VSCode and configure extension
     * Open VSCode and open the folder `FRC-team230-2024`
     * Recommended VSCode extensions
       * [Better Comments by Aaron Bond](https://marketplace.visualstudio.com/items?itemName=aaron-bond.better-comments)
       * [Doxygen Documentation Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen)
       * [Prettier](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode)
5. How to Setup vendor libraries (CTRE Phoenix, PhotonVision C++ Lib)
    * Right-click on `build.gradle` in the project tree, then select `Manage Vendor Libraries`
    * At the top of your screen, a menu will appear. Select `Install new libraries (offline)`.
    * The menu will now display a list of vendor libraries you can install. Check `CTRE Phoenix`, then click “OK”
    * Right-click on `build.gradle` in the project tree, then select `Manage Vendor Libraries`
    * At the top of your screen, a menu will appear. Select `Install new libraries (online)`.
    * To download dependency, paste:

      ```
      https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
      ```

      ```
      https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json
      ```

      ```
      https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-latest.json
      ```

      ```
      https://software-metadata.revrobotics.com/REVLib-2024.json
      ```

### Driver Station Install

*Not recommended for developer installation*

#### Prerequisites

Before installing FRC Game Tools, remove any old versions. *Click Start >> Add or Remove Programs. Locate the entry labeled “NI Software”, and select Uninstall.*

#### Install

1. [Install 2024 FRC Game Tools](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)
2. [Install CTRE Phoenix firmware](https://store.ctr-electronics.com/software/) and install *Phoenix Tuner X* on the Windows Store
3. [Install REV Robotics firmware](https://docs.revrobotics.com/rev-hardware-client/gs/install)
4. [Install FRC Radio Configuration 23.0.2](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)
    * Disable all other network adapters during use
    * [Advanced debugging](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)

</details>

## Usage

Locate the WPILib VSCode extension and select `Build Robot Code` or `Deploy Robot Code` when connected to the robot

## Contributing

### Important Notes

* Always work on your own branch, not on the main branch.

* Pull requests are mandatory to merge code into the main branch.
* Commit often and with descriptive messages.
* Don't be afraid to ask for help if you're stuck!

#### *Critical*: Code in a branch other than main

* Open the Command Palette
  * Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> (or <kbd>Cmd</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> on macOS).
* Type "Git: Create Branch" and select it.
* Enter a descriptive name for your branch (e.g., "feature/drivetrain")
* Switch to the new branch
  * Click on the current branch name in the status bar and select your new branch.

### Commit Code

1. Open the Source Control panel:

* Click on the Source Control icon in the left sidebar (it looks like a branching tree).

2. Stage your changes:
    * Select the files you want to include in your commit.
    * Click the + icon next to each file to stage them.
3. Commit your changes:
    * Write a clear and concise commit message in the top text box.
    * Click the Commit button (the checkmark icon).
    * Creating a Branch

### Merge Code with Pull Requests

1. Push your branch to GitHub:
    * Click the Sync Changes icon (the up and down arrows).
2. Create a pull request on GitHub:
    * Navigate to the repository on GitHub.
    * Click the Pull Requests tab.
    * Click the New pull request button.
    * Select your branch as the source and main as the destination.
    * Briefly note what changes you made and how breaking they are
    * Click Create pull request.
3. Get your code reviewed:
   * Addresses any bugs and potential issues

### Advanced

1. Create a local branch or check out to an existing
    * Use `git checkout` to switch to a branch or add `-b` to create a new one. Remember to commit changes before checking out
2. Stage all changes with `git add .` and commit them with `git commit -m "example commit message"`
3. Push changes to GitHub with `git push` and create a pull request to main

## Examples

Please view the example CPP and header file to see how to structure your subsystem.

### Example.cpp

```cpp
#pragma once

#include "Example.h"
#include "Robot.h"

/**
 * @brief Construct a new Sample:: Sample object
 *
 * @param pRobot pointer to the main robot object
 */
Sample::Sample(Robot *pRobot)
{
    mainRobot = pRobot;
    LocalReset();
}
/**
 * @brief Reset all member variables to default state
 *
 */
void Sample::LocalReset()
{
}
/**
 * @brief Starting configuration for the subsystem - particularly useful for initializing sensors/motors
 *
 */
void Sample::StartingConfig()
{
}
/**
 * @brief Analyze sensor data and update member variables
 *
 */
void Sample::Analyze()
{
}
/**
 * @brief Run control loops for the subsystem
 *
 */
void Sample::Service()
{
}
/**
 * @brief A function that can do anything if you believe it can do anything
 *
 */
void Sample::DoAThing()
{
}

/**
 * @brief Update Smart Dashboard or output telemetry
 *
 */
void Sample::UpdateDash()
{
}
```

### Example.h

```cpp
#pragma once

class Robot;


class Sample
{
private:
    // Create objects needed by this class
    // example: TalonFx *sampleMotor;

    Robot *mainRobot;

    // declare member variables
    // example: double height;

public:
    // Constructor
    Sample(Robot *pRobot);
    // Functions
    void LocalReset(void);
    void StartingConfig(void);

    void Analyze(void);
    void Service(void);

    void DoAThing(void);

    void UpdateDash(void);
};
``````
