# PROJECT GUIDELINES & NON-NEGOTIABLES (STRICT MODE)

## 1. THE "SURGEON" RULE (HIGHEST PRIORITY)
* **TOUCH ONLY THE TARGET:** When asked to fix a function, you must ONLY modify that specific function.
* **NO "DRIVE-BY" REFACTORING:** Do not clean up, modernize, format, or "optimize" any surrounding code. If a variable looks unused, LEAVE IT. If logic looks redundant, ASSUME IT IS NECESSARY.
* **PRESERVE LOGIC:** Do not replace manual math (e.g., custom vertex mapping) with generic library calls (e.g., `getGlobalBounds`) unless explicitly instructed. The existing code often handles edge cases (like negative width) that generic functions destroy.

## 2. ZERO ASSUMPTION POLICY (CRITICAL)
* **READ BEFORE WRITE:** Never guess variable names (e.g., `tempSelected` vs `tempSelectedObjects`). If you do not see the file content, **ASK TO SEE IT** or write the solution using "Placeholders" that the user must fill in.
* **VERIFY CONTEXT:** Before proposing a fix, you must explicitly verify: "Does this function exist? Do these members exist?" based on the provided file context.

## 3. PRESERVATION OF BEHAVIOR
* **"GEOGEBRA-STYLE" PERSISTENCE:** Intersection points must NEVER be deleted when lines move apart. They must be **HIDDEN** (`setVisible(false)`). They must **REAPPEAR** automatically.
* **REFERENCE SANITIZATION:** `sanitizeReferences()` MUST be called before deletion. Never remove this safety net.

## 4. MATH & INTERSECTIONS
* **FUZZY LOGIC:** Exact equality (`==`) is forbidden for floating-point geometry. Use `squared_distance < tolerance` or `abs(a-b) < epsilon`.
* **PERSISTENCE:** Do not delete intersection points just because the equation has no solution this frame. Cache them.

## 5. ARCHITECTURE
* **NO RENAMING:** Member variables (e.g., `m_p1`, `m_corner1`) must NOT be renamed.
* **SMART POINTERS:** Use existing `std::shared_ptr` patterns. Do not introduce `new` or raw pointer ownership.
* **SMART SNAPPING REQUIRED:** Any new tool or click-based creation must use the centralized smart snapping logic in `PointUtils` or `HandleMousePress.cpp`. Prioritize **EXISTING POINTS** over creating new ones.

## 6. USER EXPERIENCE (UX)
* **SELECTION PRIORITY:** When objects overlap (e.g., a Point on top of a Rectangle), the **Point** (or the smaller/independent object) must always be selected first.
* **TOOL CANCELLATION:** Switching tools must always clear temporary buffers (`tempSelectedObjects`, `tempVectorStartPoint`) to prevent "ghost states."