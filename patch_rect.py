import re
import os

filepath = r"c:\Users\Mario_Geometry2\Desktop\Coding\Geometry_Tool_2\HandleMousePress.cpp"

with open(filepath, 'r') as f:
    content = f.read()

# Define the pattern to find the rectangle transformation block
# We use a broad regex to match the if block and its contents regardless of specific whitespace.
pattern = r'if\s*\(!individualSuccess\s*&&\s*\(sourceShared->getType\(\)\s*==\s*ObjectType::Rectangle\s*\|\|\s*sourceShared->getType\(\)\s*==\s*ObjectType::RectangleRotatable\)\)\s*\{[^{}]*rect\s*=\s*std::dynamic_pointer_cast<Rectangle>\(sourceShared\);.*?individualSuccess\s*=\s*true;\s*\}\s*\}'

replacement = """    if (!individualSuccess && (sourceShared->getType() == ObjectType::Rectangle || sourceShared->getType() == ObjectType::RectangleRotatable)) {
      auto rect = std::dynamic_pointer_cast<Rectangle>(sourceShared);
      
      // 1. FETCH BY NAME (Crucial for correct Geometry)
      // We explicitly grab the perimeter points: BL -> BR -> TR -> TL
      auto pA = rect->getCorner1Point(); // Bottom-Left
      auto pB = rect->getCornerBPoint(); // Bottom-Right
      auto pC = rect->getCorner2Point(); // Top-Right (Diagonal)
      auto pD = rect->getCornerDPoint(); // Top-Left

      // 2. TRANSFORM
      auto tA = createTransformedPoint(editor, pA, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto tB = createTransformedPoint(editor, pB, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto tC = createTransformedPoint(editor, pC, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);
      auto tD = createTransformedPoint(editor, pD, tool, pivotPoint, lineObj, circleObj, nullptr, nullptr);

      if (tA && tB && tC && tD) {
        // 3. REGISTER
        registerTransformPoint(editor, tA);
        registerTransformPoint(editor, tB);
        registerTransformPoint(editor, tC);
        registerTransformPoint(editor, tD);

        // 4. PACK IN PERIMETER ORDER: [A, B, C, D]
        // The Rectangle constructor EXPECTS: [BottomLeft, BottomRight, TopRight, TopLeft]
        std::vector<std::shared_ptr<Point>> ordered = {tA, tB, tC, tD};

        // 5. CREATE
        auto newRect = std::make_shared<Rectangle>(
            ordered, 
            rect->isRotatable(), // Preserve Rotation Flag
            rect->getColor(),
            editor.objectIdCounter++
        );

        newRect->setThickness(rect->getThickness());
        newRect->setDependent(true); // Lock it to the points

        // 6. METADATA
        std::shared_ptr<GeometricObject> auxObj;
        if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;
        else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;
        else auxObj = pivotPoint;

        attachTransformMetadata(sourceShared, newRect, tool, auxObj, nullptr, nullptr);
        applyRectangleVertexLabels(editor, newRect, ordered);
        
        editor.addObject(newRect);
        editor.commandManager.pushHistoryOnly(std::make_shared<CreateCommand>(editor, newRect));
        individualSuccess = true;
      }
    }"""

new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)

if new_content == content:
    print("Error: Pattern not found!")
    # Try a slightly more relaxed pattern if the first one fails
    pattern_relaxed = r'if\s*\(!individualSuccess\s*&&\s*\(sourceShared->getType\(\)\s*==\s*ObjectType::Rectangle\s*\|\|\s*sourceShared->getType\(\)\s*==\s*ObjectType::RectangleRotatable\)\)\s*\{.*?individualSuccess\s*=\s*true;\s*\}\s*\}'
    new_content = re.sub(pattern_relaxed, replacement, content, flags=re.DOTALL)
    
    if new_content == content:
        print("Error: Relaxed pattern also failed!")
        exit(1)

with open(filepath, 'w') as f:
    f.write(new_content)

print("Successfully patched HandleMousePress.cpp")
