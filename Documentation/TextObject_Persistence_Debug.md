# TextObject Persistence Debug (Save/Load + Create)

## 1) Forced Type Identity (`TextObject.h`)

`TextObject` explicitly overrides `getType()`:

```cpp
ObjectType getType() const override { return ObjectType::TextObject; }
```

This guarantees runtime type dispatch through the enum path.

## 2) Loud Save Loop (`ProjectSerializer.cpp`)

The save path now scans the unified object list and logs each text object:

```cpp
json textsArray = json::array();
auto allObjects = editor.getAllObjects();
std::cout << "[DEBUG] Save scan started. Total objects in unified list: " << allObjects.size() << std::endl;
for (const auto& obj : allObjects) {
  if (!obj) continue;

  if (obj->getType() == ObjectType::TextObject) {
    std::cout << "[DEBUG] Found TextObject ID: " << obj->getID() << std::endl;
    auto textObj = std::dynamic_pointer_cast<TextObject>(obj);
    if (textObj) {
      if (!textObj->isValid()) {
        std::cout << "[DEBUG] Skipping invalid TextObject ID: " << textObj->getID() << std::endl;
        continue;
      }

      json tJson;
      tJson["type"] = "text";
      tJson["id"] = textObj->getID();
      Point_2 pos = textObj->getCGALPosition();
      tJson["x"] = CGAL::to_double(pos.x());
      tJson["y"] = CGAL::to_double(pos.y());
      tJson["content"] = textObj->getRawContent();
      tJson["isLatex"] = textObj->isRichText();
      tJson["fontSize"] = textObj->getFontSize();
      tJson["color"] = colorToHex(textObj->getColor());
      tJson["visible"] = textObj->isVisible();
      tJson["locked"] = textObj->isLocked();

      addTransformMetadata(tJson, textObj);
      textsArray.push_back(tJson);
    } else {
      std::cout << "[ERROR] Failed to cast TextObject! ID: " << obj->getID() << std::endl;
    }
  }
}
std::cout << "[DEBUG] Saved TextObject count: " << textsArray.size() << std::endl;
project["objects"]["texts"] = textsArray;
```

## 3) Creation Persistence Check (`GeometryEditor.cpp`)

After creating a text object through the command system:

```cpp
auto newText = std::make_shared<TextObject>(m_pendingTextPosition, text, isLatex, fontSize);
commandManager.execute(std::make_shared<CreateCommand>(*this, std::static_pointer_cast<GeometricObject>(newText)));
std::cout << "[DEBUG] Created TextObject. Total Objects: " << getAllObjects().size() << std::endl;
```

## Notes

- This codebase does not use a single `m_objects` container; it uses typed containers and `getAllObjects()` as the unified view.
- The debug output now confirms object discovery, cast validity, and final saved count for `objects.texts`.
