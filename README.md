[![Java CI with Gradle](https://github.com/WaterGame2023/RH-2023-Code/actions/workflows/gradle.yml/badge.svg)](https://github.com/WaterGame2023/RH-2023-Code/actions/workflows/gradle.yml)
# Team 2531 RoboHawks 2023 Robot

## Table of contents

- [Team 2531 RoboHawks 2023 Robot](#team-2531-robohawks-2023-robot)
  - [Table of contents](#table-of-contents)
  - [About](#about)
  - [Getting started](#getting-started)
    - [Prerequisites](#prerequisites)
  - [Making changes](#making-changes)
    - [Cloning the repo](#cloning-the-repo)
      - [Issues](#issues)
      - [Branches](#branches)
      - [Adding commits](#adding-commits)
      - [Pushing commits](#pushing-commits)
      - [Pulling branches](#pulling-branches)
      - [Pull requests](#pull-requests)
  - [Contact](#contact)

## About

This year, our robot consists of a Swerve Drive Specialties MK4 L1 swerve drive base with full Falcon500 motors, and we are using a 3-axis arm mounted on top, driven by two REV Neo motors and one Falcon500. On the end of that arm there is a mechanism to pick up the cones and cubes, however we do not yet know which design will be finalized for that.

## Getting started

### Prerequisites

Install the following:
- [WPILib](https://docs.wpilib.org/en/stable/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html)
- [Git](https://git-scm.com/download/win)
- [GitHub Desktop] (https://desktop.github.com/) (You don't need GitHub desktop, it is just easier to use with a smoother installation process)

## Making changes

### Cloning the repo

1. Install the [GitHub VS Code extension](vscode:extension/GitHub.vscode-pull-request-github)
2. Go to the GitHub tab and sign in using the account that has access to this repository
3. Click `Clone Repository` and search for `2531RoboHawks/2023-RobotCode`
4. Clone it into a known folder that has **no spaces** in its path
5. Open the repository in VS Code

#### Issues

[GitHub issues](https://github.com/2531RoboHawks/2023-RobotCode/issues) are how we track which tasks need work along with who should be working on them. Pay attention to the issue number since this is how each one is uniquely identified. To create a new issue, visit the [issues tab in the repository](https://github.com/2531RoboHawks/2023-RobotCode/issues) and click `New issue`. Give it a descriptive title and enough information in the comment area so that anyone working on the issue knows exactly what needs to be changed/fixed. Additionally, assignees (who is working on it) and labels (what type of issue is this) can be assigned on the right.

#### Branches

Branches are simply named pointers that point to a commit. Our main branch is called `main`, but we don't make changes to it directly. Instead, a new branch should be created before any code is modified. Click on the current branch in the bottom-left corner in VS Code and then click `+ Create new branch...`. The following naming conventions should be followed:

For a feature branch (major change):
`<yourInitials>-<feature>-<issue number>`

For a smaller, individual/small group branch:
`<yourInitials>-<issue number>-<few words describing feature>`

#### Adding commits

After finishing a small change, such as modifying a method or adding a new file, a new commit should be made immediately. Go to the `Source Control` tab on the left side of VS Code and add the changed/added/deleted files from the `Changes` dropdown to `Staging` by clicking the `+`. Once the changes are staged, Type a **descriptive** commit message and then click the `Commit` button.

#### Pushing commits

Commiting only applies the new commit locally. So to make it available to others in the repo, it needs to be pushed to the `remote` (GitHub's servers in this case). Either click `publish` in the `Source Control` tab if the branch hasn't been pushed before or `push` it otherwise to send the new commit to the remote's branch (this will be called `origin/<your branch's name>`).

#### Pulling branches

If someone else has made their branch available on the remote, you might be wondering how other people can get those same commits into their local repo. This process is called `pulling`, and it involves downloading the remote's commits and merging them with the local branch's commits.

Sometimes, you might see a `merge conflict` appear which can happen if more than one person has worked on the same line in a file, thus making git unable to automatically merge them. To fix this, click on the conflicted file (denoted by a `!`) that opens the merge conflict editor. Once at the line(s) in question, either accept the incoming (one that exists on GitHub), the HEAD (the one that you have locally), or attempt to manually merge the two together by directly editing the line(s). Repeat this process for each line/file until all conflicts are resolved. Once done, stage the now-fixed files, create a new commit, and push.

#### Pull requests

Now that all of your changes have been made (and *tested!*), it's time to get them merged into the main branch. Either click the `Create Pull Request` button in the `Source Control` tab (inline with the `SOURCE CONTROL` text, fourth one from the left) or go to the [`Pull requests` tab on GitHub](https://github.com/2531RoboHawks/2023-RobotCode/pulls), click `New pull request`, and set the `compare` branch to your branch. Fill in the template with a good title, a detailed description, a list of changes made, and the issue number before creating it. Assign a reviewer(s) on the right and label the PR accordingly.

The reviewer is responsible for looking over the PR, testing the changes themselves, and adding comments to the code changes if necessary. There are three possible actions a reviewer can take:

- Approve (PR is good; required to merge and should be done by a mentor or team lead)
- Disapprove (PR needs changes before it can be merged; please address the comments, click `Resolve` under each comment once fixed, then re-request a review)
- Comment (simply add a comment(s) without approving or disapproving)

## Contact

If you would like to contact the RoboHawks programmers, we don't currently have a way to do that.

Project Link: [2023-RobotCode](https://github.com/2531RoboHawks/2023-RobotCode)
