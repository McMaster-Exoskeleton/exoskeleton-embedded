# Contributing to McMaster Exoskeleton Embedded Repository

Thank you for your interest in contributing to the McMaster Exoskeleton embedded code repository! This document provides guidelines and instructions for contributing to this project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Testing](#testing)
- [Documentation](#documentation)

## Code of Conduct

<!-- TODO: Add your project's code of conduct or link to an existing one -->

This project adheres to a code of conduct. By participating, you are expected to uphold this code.

## Getting Started

### Prerequisites

<!-- TODO: List required tools and versions -->
- [ ] Embedded development toolchain (e.g., ARM GCC, STM32CubeIDE, etc.)
- [ ] Hardware debugging tools (e.g., ST-Link, J-Link, etc.)
- [ ] CAN bus interface (if applicable)
- [ ] Git version control

### Setting Up Development Environment

<!-- TODO: Add specific setup instructions -->
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd exoskeleton-embedded
   ```

2. <!-- TODO: Add build system setup instructions -->
   - Configure build system
   - Install dependencies
   - Set up hardware connections

3. <!-- TODO: Add verification steps -->
   - Verify build process
   - Test hardware communication

## Development Workflow

### Branch Naming Convention

<!-- TODO: Customize branch naming convention -->
- `feature/description` - New features
- `bugfix/description` - Bug fixes
- `refactor/description` - Code refactoring
- `docs/description` - Documentation updates
- `test/description` - Test additions/updates

### Creating a Branch

```bash
git checkout -b feature/your-feature-name
```

## Coding Standards

### C++ Style Guide

<!-- TODO: Specify your C++ standard and style preferences -->
- **C++ Standard**: C++11 or later (specify your target)
- **Naming Conventions**:
  - Classes: `PascalCase` (e.g., `MotorController`)
  - Functions: `camelCase` (e.g., `readSensor()`)
  - Variables: `snake_case` or `camelCase` (be consistent)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_MOTORS`)
  - Private members: `snake_case_` with trailing underscore

### Code Formatting

<!-- TODO: Specify formatting tool and configuration -->
- Use consistent indentation (spaces or tabs - specify)
- Maximum line length: 80 or 100 characters (specify)
- Use clang-format or similar tool (specify configuration)

### Documentation

- All public functions must have Doxygen-style comments
- Include `@brief`, `@param`, `@return` tags where applicable
- Document complex algorithms and hardware-specific code
- Keep comments up-to-date with code changes

### Example

```cpp
/**
 * @brief Read sensor data from specified sensor
 * @param sensor_id Sensor identifier (0 to MAX_SENSORS-1)
 * @param data Pointer to data buffer to fill
 * @return true if read successful, false otherwise
 */
bool readSensor(uint8_t sensor_id, SensorData* data);
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
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Test additions/updates
- `chore`: Build process or auxiliary tool changes

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

1. **Update Documentation**: Ensure all code changes are documented
2. **Update Tests**: Add or update tests for new functionality
3. **Check Build**: Ensure code compiles without warnings
4. **Test Hardware**: Test on actual hardware if applicable
5. **Create PR**: 
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

- Use Doxygen for API documentation
- Keep README.md updated with build and usage instructions
- Document hardware requirements and connections

### Inline Comments

- Explain "why" not "what" in comments
- Document hardware-specific workarounds
- Note timing constraints and real-time requirements

## Hardware-Specific Guidelines

<!-- TODO: Add hardware-specific guidelines -->
- **Target Platform**: <!-- Specify (e.g., STM32F4, ESP32, etc.) -->
- **Real-time Constraints**: <!-- Document timing requirements -->
- **Memory Constraints**: <!-- Document memory limitations -->
- **Safety Considerations**: <!-- Document safety-critical aspects -->

## Questions?

<!-- TODO: Add contact information or communication channels -->
If you have questions, please:
- Open an issue for discussion
- Contact the maintainers at: <!-- Add contact info -->
- Check existing issues and discussions

## License

<!-- TODO: Specify license and contribution agreement -->
By contributing, you agree that your contributions will be licensed under the same license as the project.

---

Thank you for contributing to the McMaster Exoskeleton project!

