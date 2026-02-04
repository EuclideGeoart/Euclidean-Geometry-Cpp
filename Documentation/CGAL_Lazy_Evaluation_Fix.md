# Fix for CGAL Lazy Evaluation Stack Overflow in Shape Updates

## Problem

When updating geometric shapes (Polygon, RegularPolygon, Triangle, Rectangle) in interactive scenarios, the use of CGAL's `Lazy_construction` kernel led to stack overflow crashes. This was due to the accumulation of deep lazy evaluation histories in `Point_2` objects, especially when shapes were transformed or updated repeatedly.

Additionally, a recursive notification loop was discovered:
- `Shape::updateDependentShape` → `Point::setCGALPosition` → `updateHostedPoints` → `Shape::updateDependentShape` ...
This caused infinite recursion and stack overflow during dependent shape updates.

## Solution

### 1. Flattening Point Assignments
All assignments to `Point_2` in shape update methods now use a `flattenPoint` utility, which converts the point to double and back, breaking the lazy evaluation chain and preventing deep construction histories.

### 2. Constraint Update Deferral
During dependent shape updates, constraint updates on all involved points are deferred using `setDeferConstraintUpdates(true)`. After all position updates, `forceConstraintUpdate()` is called to flush and apply the updates. This prevents recursive notification loops and stack overflows.

#### Example Pattern (in `updateDependentShape`):
```
for (auto* pt : points) pt->setDeferConstraintUpdates(true);
// ... update all point positions ...
for (auto* pt : points) pt->forceConstraintUpdate();
```

### 3. Affected Classes and Methods
- **Polygon, RegularPolygon, Triangle, Rectangle**: All update/assignment sites in `updateDependentShape`, `translate`, and related methods now use flattening and constraint update deferral/flush.
- **Point**: Constraint update logic supports deferral and explicit flush.

## Outcome
- No more stack overflow or infinite recursion during interactive shape updates.
- All geometric shape classes are robust against CGAL lazy evaluation history growth and notification recursion.

## Commit Details
- All changes are committed with a message referencing this documentation.

---
*Date: 2026-02-04*
*Author: GitHub Copilot*
