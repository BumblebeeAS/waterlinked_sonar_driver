# Contributing

## Development setup

Follow the setup in the [README](README.md), then install [pre-commit](https://pre-commit.com/) and `clang-tidy`:

```bash
pre-commit install
colcon build --packages-up-to waterlinked_sonar_driver --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select waterlinked_sonar_driver
colcon test-result --verbose
```

## Style

- Formatting is enforced through pre-commit: `clang-format` for C++, `ruff` for Python, `gersemi` for CMake.
- `clang-tidy` checks are configured in `.clang-tidy`; run them with `clang-tidy -p build/waterlinked_sonar_driver src/waterlinked_sonar_driver.cpp`.

## Pull requests

- Open pull requests against `main`. CI must pass.
- Follow [Conventional Commits](https://www.conventionalcommits.org/) for commit messages (`feat:`, `fix:`, `docs:`, `ci:`, `chore:`).
- Note user-visible changes under `Unreleased` in [CHANGELOG.md](CHANGELOG.md).
