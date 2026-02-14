# Bug Fix: Resurrection and Intersection Labeling Persistence

## Issue Summary
1. **The "Coma" Bug**: Dependent objects (like lines connected to intersection points) remained hidden even after their parent intersection points became valid again.
2. **Intersection Labeling Glitch**: Every time an intersection point disappeared and reappeared, it was treated as a new object, gaining a new ID and a new label (e.g., changing from 'E' to 'F' to 'G'), which also broke any lines connected to it.

## Fix Details

### 1. Robust Resurrection Logic (`Line.cpp`)
Modified `Line::update` to implement an automatic visibility management system based on parent health. 
- **Auto-Hide**: If endpoints become invalid (e.g., intersection lines no longer cross), the line hides itself.
- **Auto-Resurrect**: If endpoints become valid again, the line automatically restores its visibility.
- **Construction Awareness**: Logic preserves visibility for Perpendicular and Parallel tool lines even when their helper points are hidden.

```cpp
// Logic in Line::update
bool parentsAreValid = true;
if (!m_startPoint || !m_startPoint->isValid()) parentsAreValid = false;
if (!m_endPoint || !m_endPoint->isValid()) parentsAreValid = false;

// Special check for constrained lines (Parallel/Perpendicular)
if (m_isParallelLine || m_isPerpendicularLine) {
    auto refObj = m_constraintRefObject.lock();
    if (refObj && !refObj->isValid()) parentsAreValid = false;
}

if (parentsAreValid) {
    if (!this->isVisible()) setVisible(true); // RESURRECTION
} else {
    if (this->isVisible()) setVisible(false); // HIDE
    return;
}
```

### 2. Guaranteed Point Reuse (`Intersection.cpp`)
Rewrote the intersection point matching system. Instead of relying on proximity matching (which fails when coordinates become invalid/NaN), the system now guarantees the reuse of existing points as long as the constraint exists.
- **Identity Preservation**: Connected lines keep their references to the same point object.
- **Label Persistence**: Labels and IDs are preserved across validity cycles.
- **Stale Point Management**: Invalid points are kept in a "dormant" state within the constraint to ensure they are available for reuse when the geometry recovers.

```cpp
// Logic in DynamicIntersection::updateAllIntersections
size_t existingIdx = 0;
for (const auto &p : newIntersections) {
    std::shared_ptr<Point> reusePoint = nullptr;
    while (existingIdx < existingCount) {
        if (!used[existingIdx] && existing[existingIdx]) {
            reusePoint = existing[existingIdx];
            used[existingIdx] = true;
            existingIdx++;
            break;
        }
        existingIdx++;
    }

    if (reusePoint) {
        reusePoint->setVisible(true);
        reusePoint->setIsValid(true);
        reusePoint->setCGALPosition(p);
        reusePoint->update();
        reusePoint->updateConnectedLines(); // Notify children to resurrect
    } else {
        // Create new only if absolutely necessary
    }
}
```

### 3. Observer Registration Fix (`Line.cpp`)
Fixed a critical bug where lines were not registering as observers of their `m_endPoint`. 
- **Fix**: Uncommented `m_endPoint->addConnectedLine(shared_from_this())` in `registerWithEndpoints()`.
- **Result**: Lines now receive instantaneous notifications when *either* endpoint updates its state, ensuring immediate resurrection.

## Conclusion
The combination of guaranteed object reuse and proactive state notification ensures a mathematically stable and visually consistent experience for complex geometric constructions.
