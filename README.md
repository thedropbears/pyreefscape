# py2025

The Drop Bears' robot code for FRC 2025

## Setup

### Install dependencies

We use `uv` to manage our dependencies in our development environments.
This includes the Python version, and any Python packages such as `wpilib`.

Install `uv` by following the [`uv` docs](https://docs.astral.sh/uv/).

After installing `uv`, use it to create a virtual environment and install our dependencies.

```sh
uv sync
```

Then, download the roboRIO dependencies.

```sh
uv run python -m ensurepip
uv run robotpy sync --no-install
```

### pre-commit

[pre-commit][] is configured to run our formatters and linters.
These are enforced for all code committed to this project.

To use pre-commit, you must install it outside of this project's virtual environment.
Either use your system package manager, or use `uv tool`:

```sh
uv tool install pre-commit
```

You can then set up the pre-commit hooks to run on commit:

```sh
pre-commit install
```

[pre-commit]: https://pre-commit.com

### IDE setup

#### Visual Studio Code

1. Install the workspace's recommended extensions.
2. Copy `.vscode/settings.default.json` to `.vscode/settings.json`.

   ```sh
   cp .vscode/settings.default.json .vscode/settings.json
   ```

## Run

### Simulation

Before your first run, copy the `*.json.orig` files to the main directory and remove the `.orig` extension.

```
uv run robotpy sim
```

### Deploy to Robot

Once on robots network

```
uv run robotpy deploy
```

### Test

```
uv run robotpy test
```

### Type checking

We use mypy to check our type hints in CI. You can run mypy locally:

```sh
uv run mypy .
```

## Code Structure

We use RobotPy's Magicbot framework

`robot.py`: Entry point, has mapping from driver inputs to high level robot actions.

`components/`: Abstracts hardware into robot actions.

`controllers/`: Automates robot actions, mostly with state machines.

`autonomous/`: Controls robot during autonomous period.

`ids.py`: Has CAN ids, PH channels and other port numbers.
