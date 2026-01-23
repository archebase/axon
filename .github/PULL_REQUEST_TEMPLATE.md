## Pull Request Checklist

Please ensure your PR meets the following requirements:

- [ ] Code follows the [style guidelines](CONTRIBUTING.md#code-style)
- [ ] Tests pass locally (`make test` or `make docker-test`)
- [ ] Code is formatted (`make format`)
- [ ] Documentation updated if needed
- [ ] Commit messages follow [conventional commits](CONTRIBUTING.md#commit-message-format)
- [ ] PR description is complete and clear

---

## Summary

<!-- Provide a concise 1-2 sentence description of the changes -->

This PR adds/fixes/changes [brief description].

---

## Motivation

<!-- Explain why this change is needed. What problem does it solve? -->

-

---

## Changes

<!-- Describe the technical changes in detail. Include file links with line numbers where relevant. -->

### Modified Files

- `[file path](file path)` - Description of changes
- `[file path](file path)` - Description of changes

### Added Files

- `[file path](file path)` - Description of new file

### Deleted Files

- `[file path](file path)` - Reason for deletion

---

## Type of Change

<!-- Mark the relevant option with an "x" -->

- [ ] **Bug fix** (non-breaking change that fixes an issue)
- [ ] **New feature** (non-breaking change that adds functionality)
- [ ] **Breaking change** (fix or feature that would cause existing functionality to change)
- [ ] **Documentation update** (documentation changes only)
- [ ] **Refactoring** (code improvement without functional changes)
- [ ] **Performance improvement** (code changes that improve performance)
- [ ] **Test changes** (adding, modifying, or removing tests)

---

## Impact Analysis

### Breaking Changes

<!-- If this is a breaking change, describe what users need to do to adapt -->

None

### Backward Compatibility

<!-- Describe any backward compatibility implications -->

Fully backward compatible

---

## Testing

### Test Environment

- ROS Distribution: [e.g., ROS 2 Humble / ROS 1 Noetic]
- OS: [e.g., Ubuntu 22.04]
- Build Type: [e.g., Release / Debug]

### Test Cases

<!-- List the specific test cases you ran -->

- [ ] Unit tests pass locally
- [ ] Integration tests pass locally
- [ ] E2E tests pass (if applicable)
- [ ] Manual testing completed

#### Manual Testing Steps

<!-- If manual testing was performed, describe the steps -->

1. `./axon_recorder --config test_config.yaml`
2. `curl -X POST http://localhost:8080/rpc/config -d '{...}'`
3. Observed expected behavior: ...

### Test Coverage

<!-- Indicate if new tests were added -->

- [ ] New tests added
- [ ] Existing tests updated
- [ ] Coverage maintained or improved

---

## Screenshots / Recordings

<!-- Include screenshots or recordings if applicable (especially for UI/UX changes) -->

<!-- Before: [screenshot] -->
<!-- After: [screenshot] -->

---

## Performance Impact

<!-- Describe any performance implications of this change -->

- Memory usage: [No change / Increased / Decreased]
- CPU usage: [No change / Increased / Decreased]
- Recording throughput: [No change / Improved / Regressed]
- Lock contention: [No change / Reduced / Increased]

---

## Documentation

<!-- Link to any documentation updates -->

- [ ] [README.md](README.md) updated
- [ ] [ARCHITECTURE.md](ARCHITECTURE.md) updated
- [ ] [CONTRIBUTING.md](CONTRIBUTING.md) updated
- [ ] API documentation updated
- [ ] Design docs updated in [docs/designs/](docs/designs/)
- [ ] No documentation changes needed

---

## Related Issues

<!-- Link to related issues using keywords (e.g., "fixes #123", "closes #456", "related to #789") -->

- Fixes #
- Related to #
- Refers to #

---

## Additional Notes

<!-- Any additional information that reviewers should know -->

-

---

## Reviewers

<!-- Tag specific reviewers if needed -->

@reviewer1 @reviewer2

---

### Notes for Reviewers

<!-- Highlight specific areas you'd like reviewers to focus on -->

- Please review the changes to `[specific file]`
- Focus on the thread safety of the new implementation
- Verify the plugin ABI changes are backward compatible

---

## Checklist for Reviewers

<!-- For reviewers to check -->

- [ ] Code changes are correct and well-implemented
- [ ] Tests are adequate and pass
- [ ] Documentation is updated and accurate
- [ ] No unintended side effects
- [ ] Performance impact is acceptable
- [ ] Backward compatibility maintained (if applicable)
