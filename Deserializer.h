#pragma once

#include <string>

class GeometryEditor;

class Deserializer {
 public:
  static bool loadProject(GeometryEditor& editor, const std::string& filepath);
};
