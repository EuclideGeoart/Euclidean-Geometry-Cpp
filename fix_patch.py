import re
import os

filepath = r"c:\Users\Mario_Geometry2\Desktop\Coding\Geometry_Tool_2\HandleMousePress.cpp"

with open(filepath, 'r') as f:
    lines = f.readlines()

# We know the first occurrence starts at line 1229 (approx) and the second at 1682 (approx)
# Based on findstr, the comment "FETCH BY NAME" is at 1232 and 1682.

# Fix the FIRST block (Translation: 1229+)
# Line 1232 is approx 4 lines into the block.
# We'll search for the block that uses v1, v2 as arguments to createTransformedPoint

# Actually, let's just use the line numbers carefully now that we know them.
# The findstr showed 1232 and 1682.

new_lines = []
for i, line in enumerate(lines):
    # If we are in the translation block (roughly 1230-1280)
    if 1232 <= i + 1 <= 1270:
        # replace pivotPoint, lineObj, circleObj, nullptr, nullptr
        # with nullptr, nullptr, nullptr, v1, v2
        line = line.replace("pivotPoint, lineObj, circleObj, nullptr, nullptr", "nullptr, nullptr, nullptr, v1, v2")
        # replace auxObj with vectorAux in metadata
        if "attachTransformMetadata" in line:
            line = line.replace("auxObj", "vectorAux")
        # replace creation of auxObj
        if "if (tool == ObjectType::ReflectAboutLine) auxObj = lineObj;" in line:
            line = "// Aux object handled via vectorAux in translation block\n"
        if "else if (tool == ObjectType::ReflectAboutCircle) auxObj = circleObj;" in line:
            line = ""
        if "else auxObj = pivotPoint;" in line:
            line = ""
            
    new_lines.append(line)

with open(filepath, 'w') as f:
    f.writelines(new_lines)

print("Successfully corrected translation block in HandleMousePress.cpp")
