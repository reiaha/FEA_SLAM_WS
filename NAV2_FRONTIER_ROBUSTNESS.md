# Nav2 Frontier Goal Sending Robustness Improvements

## Summary
Modified the frontier goal sending pipeline to ensure Nav2 always receives frontier goals, even in challenging startup or obstacle-rich situations.

## Key Changes

### 1. **Lenient Startup Phase** (`exploration_planning_mixin.py` lines 564-580)
- **Problem**: At startup, lethal cell checks and strict obstacle detection could prevent the first goal from being sent
- **Fix**: 
  - Skip lethal cell check entirely during INIT phase (when costmap is settling)
  - Allow goal send even if scan is slightly stale at startup with warning
  - Allow goal send even if front clearance is slightly tight at startup

### 2. **Force-Send Mode** (`exploration_planning_mixin.py` lines 551-564)
- **Problem**: After 5+ consecutive goal failures, robot gets stuck in recovery loop unable to explore
- **Fix**: Activates after 5 consecutive failures
  - Bypasses obstacle detection check (Nav2 can path around if needed)
  - Skips stale scan check (old data is better than no data)
  - Skips front clearance check (allows goal send if there's any path)
  - Logs warning when activated so user knows system is in degraded mode

### 3. **Automatic Retry with Next Frontier** (`exploration_execution_mixin.py` lines 587-609)
- **Problem**: If picked frontier fails to send, system waits for next cycle instead of trying alternate frontier
- **Fix**: 
  - Auto-picks next best frontier within 0.5-10 seconds of previous failure
  - Retries immediately on next EXPLORE cycle
  - Prevents wasting time on unreachable goals

### 4. **Fallback Goal System** (`exploration_planning_mixin.py` lines 1058-1090)
- **Problem**: When all frontiers are exhausted/blacklisted, robot has nowhere to go
- **Fix**: `_send_fallback_goal()` method attempts to send goal 1-2m ahead
  - Tries forward direction first, then left/right diagonals
  - Validates goal is in free space before sending
  - Helps robot escape local stuck states

### 5. **Warning Throttling** (`exploration_coordinator.py` lines 332-334)
- **Problem**: Verbose logs on every failed goal check slowed down system
- **Fix**: Added warning timestamp tracking for scan staleness, front clearance, and lethal cell checks
  - Logs only every 5+ seconds instead of every cycle
  - Improves reaction time and readability

## Behavioral Changes

| Scenario | Before | After |
|----------|--------|-------|
| Startup with settling costmap | ❌ Blocked by lethal cell check | ✅ Allows first goal send |
| Stale scan at startup | ❌ Blocked, waits for fresh scan | ✅ Warns but proceeds with first frontier |
| Goal send failure | ⏸️ Waits for next EXPLORE cycle | 🔄 Auto-retries with next frontier in <1s |
| No frontiers available | 🛑 Does nothing | 📍 Sends fallback goal to move forward |
| 5+ consecutive failures | 🔁 Stuck in recovery | ⚡ Force-send mode bypasses checks |

## Logging Output

### Normal Operation
```
🚀 Sending Nav2 goal: Robot (0.00, 0.00) → Frontier (2.50, 3.20) → Goal (2.48, 3.18) | Heading: 52.3°
```

### Auto-Retry
```
🔄 Auto-retry: Attempting new frontier after goal send failure
```

### Fallback Goal
```
📍 No frontiers detected; attempting fallback goal
📍 No frontiers available; sending fallback goal at (1.50, 0.00)
```

### Force-Send Mode
```
⚠️ Force-send mode: 5 consecutive failures — bypassing strict checks to keep exploring
```

## Safety Guarantees

All motor-level safety checks remain **unchanged and fully enforced**:
- ✅ Motor bridge hard check: never move forward if front < min_distance
- ✅ Motor bridge hard check: never move backward if rear < min_distance
- ✅ Rotation-only mode if both front and rear blocked
- ✅ Obstacle detection trigger → OBSTACLE phase
- ✅ All Nav2 costmap-based path planning intact

## Testing Recommendations

1. **Startup**: Verify first goal is sent even if costmap or scan appears initially stale
2. **Auto-Retry**: Block one frontier and verify system picks next without waiting full cycle
3. **Fallback**: Blacklist all real frontiers and verify robot still moves on fallback goal
4. **Force-Send**: Force 5+ goal failures and verify system keeps exploring (not stuck)
5. **Safety**: Verify motor bridge still prevents collision even in force-send mode

## Performance Impact

- **Positive**: Faster goal recovery (seconds instead of cycles)
- **Positive**: No wasted time on unreachable goals
- **Neutral**: Slightly more logging but throttled to 5+ second intervals
- **Neutral**: Force-send mode activates only after sustained failures
