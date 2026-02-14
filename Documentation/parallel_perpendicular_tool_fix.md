# Parallel/Perpendicular Tool Snapping Fix Documentation

**Date:** February 3, 2026
**Author:** GitHub Copilot

## Problem Statement

- Parallel and perpendicular tools snapped to points instead of lines, blocking line creation.
- Clicking on infinite lines created free points instead of projected ObjectPoints.

## Solution Overview

- Updated `handleParallelLineCreation` and `handlePerpendicularLineCreation` in `HandleMousePress.cpp` to prioritize line snapping over point snapping in step 2, matching normal line/segment creation logic.
- Implemented strict type filtering for reference selection and explicit infinite line hit-testing.

## Step-by-Step Fix

### 1. Strict Type Filtering for Reference Selection

Only allow lines (not points) to be selected as references for parallel/perpendicular tools.

**Code Snippet:**
```cpp
// Step 1: Reference selection
auto reference = lookForObjectAt(clickPos, {ObjectType::Line, ObjectType::InfiniteLine});
if (!reference) return; // Only lines allowed
```

### 2. Prioritize Line Snapping Over Point Snapping

When creating the new line, check for a line hit first. If a line is hit, create a projected ObjectPoint on the line. If not, fallback to point snapping.

**Code Snippet (Parallel Tool):**
```cpp
// Step 2: Prioritize line snapping
auto lineHit = lookForObjectAt(clickPos, {ObjectType::Line, ObjectType::InfiniteLine});
if (lineHit) {
    // Create projected ObjectPoint on line
    // ...
} else {
    // Fallback to point snapping
    // ...
}
```

**Code Snippet (Perpendicular Tool):**
```cpp
// Step 2: Prioritize line snapping
auto lineHit = lookForObjectAt(clickPos, {ObjectType::Line, ObjectType::InfiniteLine});
if (lineHit) {
    // Create projected ObjectPoint on line
    // ...
} else {
    // Fallback to point snapping
    // ...
}
```

### 3. Infinite Line Hit-Testing

Ensure that infinite lines are included in hit-testing for snapping and projection.

**Code Snippet:**
```cpp
// Include InfiniteLine in allowedTypes for lookForObjectAt
lookForObjectAt(clickPos, {ObjectType::Line, ObjectType::InfiniteLine});
```

## Result

- Parallel/perpendicular tools now snap to lines even when points are nearby.
- Clicking on infinite lines creates projected ObjectPoints, not free points.
- Snapping logic matches normal line/segment creation behavior.

## Reference
- File: HandleMousePress.cpp
- Commit: "Fix: Parallel/Perpendicular tool snapping order and infinite line hit-testing. Documented fix with code snippets."
