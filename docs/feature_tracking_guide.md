# Feature Tracking Guide

## Quick Start

This guide explains how to use the `feature_tracking.md` document effectively for team collaboration.

## Document Structure

The feature tracking document is organized by board:
- **Daughter Board** - Individual cell monitoring boards
- **Secondary Board** - Data aggregation board
- **Primary Board** - Main BMS controller
- **Cross-Board Features** - Features spanning multiple boards

## Status Workflow

```
⚪ TODO → 🟡 In Progress → 🟣 Review → 🟢 Completed
                ↓
            🔴 Blocked
```

### When to Update Status

1. **Start Work**: Change ⚪ → 🟡 when you begin working on a feature
2. **Ready for Review**: Change 🟡 → 🟣 when code is complete and needs review
3. **Complete**: Change 🟣 → 🟢 after review and testing pass
4. **Blocked**: Change any status → 🔴 when blocked, add blocker details in Notes

## Priority Guidelines

### P0 - Critical
- Blocks other work or core functionality
- Must be completed before system can function
- Examples: Basic communication, core data collection

### P1 - High
- Important for system functionality
- Should be completed soon
- Examples: Fault detection, data validation

### P2 - Medium
- Important but not blocking
- Can be deferred if needed
- Examples: Debug features, optimization

### P3 - Low
- Nice to have
- Can be deferred indefinitely
- Examples: Future enhancements, nice-to-have features

## Assignment Best Practices

### Taking Ownership

1. **Check Dependencies**: Ensure all dependencies are complete
2. **Update Document**: Add your name to the Owner column
3. **Update Status**: Change status to 🟡 In Progress
4. **Communicate**: Let team know you're working on it

### Reassigning Tasks

1. **Update Owner**: Change owner name in document
2. **Add Note**: Document why reassignment occurred
3. **Update Status**: Reset to appropriate status if needed
4. **Communicate**: Notify previous and new owner

## Notes Column Usage

Use the Notes column to document:
- **Blockers**: What's preventing progress
- **Decisions**: Important technical decisions made
- **Issues**: Problems encountered and solutions
- **Dependencies**: Additional dependencies not listed
- **Timeline**: Expected completion dates
- **Resources**: Links to relevant docs, PRs, or issues

## Dependency Management

### Before Starting Work

1. Check the Dependencies column
2. Verify all dependencies are 🟢 Completed
3. If dependencies are incomplete, either:
   - Wait for dependencies to complete
   - Work on dependencies first
   - Document why you can proceed anyway

### Adding Dependencies

When adding a new feature:
1. Identify what it depends on
2. List dependencies in the Dependencies column
3. Use feature names or component names
4. Update dependent features if needed

## Team Workflow

### Daily Standups

During standups, reference the tracking document:
- What did you complete? (Update ⚪ → 🟢)
- What are you working on? (Check 🟡 items)
- Any blockers? (Update to 🔴, add notes)

### Weekly Reviews

Weekly, review the document:
- Update statuses for completed work
- Identify items stuck in 🟡 for too long
- Reassign blocked items if needed
- Prioritize upcoming work

### Sprint Planning

Use the document to:
- Identify P0 and P1 items for sprint
- Assign owners based on expertise
- Estimate effort (add to Notes if needed)
- Set sprint goals

## Example Workflow

### Starting a New Feature

1. Find feature in tracking document
2. Check dependencies are complete
3. Add your name to Owner column
4. Change status: ⚪ → 🟡
5. Add note: "Starting implementation, ETA: [date]"

### Completing a Feature

1. Finish implementation
2. Update status: 🟡 → 🟣
3. Add note: "Ready for review, PR: #[number]"
4. Request review from team
5. After review passes: 🟣 → 🟢
6. Add note: "Completed and tested on [date]"

### Encountering a Blocker

1. Update status: Current → 🔴
2. Add detailed note about blocker
3. Notify team in communication channel
4. Update when blocker is resolved

## Tips for Effective Tracking

### Keep It Updated
- Update status as you work
- Don't let items sit in 🟡 for weeks
- Regular updates help team visibility

### Be Specific in Notes
- Instead of "blocked", write "blocked on hardware delivery, ETA: [date]"
- Instead of "working on it", write "implementing CAN frame generation, 50% complete"

### Use Consistent Naming
- Use same feature names across documents
- Reference related features in Notes
- Link to relevant documentation

### Regular Cleanup
- Remove completed items from "Known Issues"
- Archive completed features if document gets long
- Update priority if circumstances change

## Integration with Other Tools

### Git Workflow
- Create branch: `feature/[feature-name]`
- Reference feature in commit messages
- Link PR to feature in Notes column

### Issue Tracking
- Create GitHub/GitLab issues for complex features
- Link issues in Notes column
- Update status when issue is closed

### Documentation
- Link to relevant docs in Notes
- Update docs when feature is complete
- Reference tracking doc in code comments

## Troubleshooting

### Feature Stuck in Progress
- Check Notes for last update
- Reach out to owner
- Consider reassigning if owner unavailable
- Break down into smaller tasks if too large

### Missing Dependencies
- Review Dependencies column
- Check if dependencies are actually needed
- Update if dependencies changed
- Document why dependencies aren't needed

### Unclear Ownership
- Review team member expertise
- Check current workload
- Assign based on priority and availability
- Document assignment rationale

## Questions?

If you have questions about:
- **What to track**: Track anything that takes significant time or affects others
- **How detailed**: Be detailed enough for team to understand status
- **When to update**: Update whenever status changes, at minimum daily
- **Who can update**: Anyone on the team can update, but communicate changes

---

**Remember**: This document is a tool to help the team, not a burden. Keep it simple, keep it updated, and it will be valuable for everyone.

