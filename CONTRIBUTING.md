# Contributing to McMaster Exoskeleton Embedded Repository

## Table of Contents
- [Code of Conduct](#code-of-conduct)

- [Getting Started](#getting-started)

- [Development Workflow](#development-workflow)

- [Coding Standards](#coding-standards)

- [Commit Guidelines](#commit-guidelines)

- [Pull Request Process](#pull-request-process)

- [Testing](#testing)

- [Documentation](#documentation)


## Getting Started

### Setting Up Development Environment
<!-- TODO: Add specific setup instructions -->

1. Clone the repository:

```bash

git clone https://github.com/McMaster-Exoskeleton/exoskeleton-embedded.git

cd exoskeleton-embedded

```

  

2.  <!-- TODO: Add build system setup instructions -->

- Configure build system (TBD)

- Install dependencies (TBD)

- Set up hardware connections (Document with wiring to be created)

  

3.  <!-- TODO: Add verification steps -->

- Verify build process

- Test hardware communication

  

## Development Workflow

  

### Branch Naming Convention

  

<!-- TODO: Customize branch naming convention -->

-  `feature/description` - New features

-  `bugfix/description` - Bug fixes

-  `refactor/description` - Code refactoring

-  `docs/description` - Documentation updates

-  `test/description` - Test additions/updates

  

### Creating a Branch

  

```bash

git  checkout  -b  feature/your-feature-name

```

  

## Coding Standards

  

### C++ Style Guide

  

<!-- TODO: Specify your C++ standard and style preferences -->

-  **Naming Conventions**:

- Classes: `PascalCase` (e.g., `MotorController`)

- Functions: `camelCase` (e.g., `readSensor()`)

- Variables: `snake_case` or `camelCase` (be consistent)

- Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_MOTORS`)

- Private members: `snake_case_` with trailing underscore

  

### Code Formatting

  

<!-- TODO: Specify formatting tool and configuration -->

- Use consistent indentation ( indent within loops, conditional statements, classes, structs, etc.)

- Maximum line length: 100 characters

  

### Documentation

- Include `@brief`, `@param`, `@return` tags where applicable

- Keep documentation for each class and it's methods in their respective file

- Keep comments up-to-date with code changes

  

### Example

  

```cpp

/**

* @brief Read sensor data from specified sensor

* @param  sensor_id Sensor identifier (0 to MAX_SENSORS-1)

* @param  data Pointer to data buffer to fill

* @return true if read successful, false otherwise

*/

bool  readSensor(uint8_t  sensor_id, SensorData*  data);

```

  

## Commit Guidelines

  

### Commit Message Format

  

<!-- TODO: Customize commit message format -->

Follow conventional commits format:

  

```

<type>(<scope>): <subject>

  

<body>

  

<footer>

```

  

**Types**:

-  `feat`: New feature

-  `fix`: Bug fix

-  `docs`: Documentation changes

-  `style`: Code style changes (formatting, etc.)

-  `refactor`: Code refactoring

-  `test`: Test additions/updates

-  `chore`: Build process or auxiliary tool changes

  

**Examples**:

```

feat(motors): add position control mode

  

Implement PID position control for motor drivers with configurable

gains and limits.

  

Closes #123

```

  

```

fix(canbus): handle extended frame format correctly

  

Fix bug where extended CAN frames were not being parsed correctly,

causing message loss.

```

  

## Pull Request Process

  

1.  **Update Documentation**: Ensure all code changes are documented

2.  **Update Tests**: Add or update tests for new functionality

3.  **Check Build**: Ensure code compiles without warnings

4.  **Test Hardware**: Test on actual hardware if applicable

5.  **Create PR**:

- Use descriptive title

- Reference related issues

- Describe changes in detail

- Include testing instructions

  

### PR Checklist

  

<!-- TODO: Customize checklist -->

- [ ] Code follows style guidelines

- [ ] Self-review completed

- [ ] Comments added for complex code

- [ ] Documentation updated

- [ ] No new warnings generated

- [ ] Hardware tested (if applicable)

- [ ] Tests added/updated

- [ ] All tests pass

  

## Testing

  

### Unit Testing

  

<!-- TODO: Specify testing framework and approach -->

- Use appropriate testing framework for embedded code

- Test individual functions and classes

- Mock hardware dependencies where possible

  

### Hardware Testing

  

<!-- TODO: Specify hardware testing requirements -->

- Test on target hardware before submitting PR

- Document hardware configuration used

- Include test results in PR description

  

### Test Coverage

  

<!-- TODO: Specify coverage requirements -->

- Aim for high test coverage on critical code paths

- Focus on safety-critical functions

  

## Documentation

  

### Code Documentation

  

- Create API documentation

- Keep README.md updated with build and usage instructions

- Document hardware requirements and connections

  

### Inline Comments

  

- Explain "why" not "what" in comments

- Document hardware-specific workarounds

- Note timing constraints and real-time requirements

  

## Hardware-Specific Guidelines
The following will be filled out as requirements become more clear
  

<!-- TODO: Add hardware-specific guidelines -->

-  **Target Platform**: <!-- Specify (e.g., STM32F4, ESP32, etc.) -->

-  **Real-time Constraints**: <!-- Document timing requirements -->

-  **Memory Constraints**: <!-- Document memory limitations -->

-  **Safety Considerations**: <!-- Document safety-critical aspects -->