// GenericDeleteCommand.h
#pragma once
#include "Command.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>


// This templated command works on any type T stored in a container (vector of
// unique_ptr<T>)
template <typename T> class GenericDeleteCommand : public Command {
public:
  GenericDeleteCommand(std::vector<std::unique_ptr<T>> &container, int index)
      : container(container), index(index) {
    if (index < 0 || index >= static_cast<int>(container.size())) {
      throw std::out_of_range(
          "Index is out of range in GenericDeleteCommand constructor");
    }
  }

  void execute() override {
    try {
      // Verify index before use
      if (index < 0 || index >= static_cast<int>(container.size())) {
        throw std::out_of_range(
            "Index is out of range in GenericDeleteCommand::execute");
      }

      // Save the item (via move) so that we can restore it later.
      removedItem = std::move(container[index]);
      container.erase(container.begin() + index);
    } catch (const std::exception &e) {
      std::cerr << "Error in GenericDeleteCommand::execute: " << e.what()
                << std::endl;
      throw; // Re-throw the exception
    }
  }

  void undo() override {
    try {
      // Check index against container size for robustness.
      if (index < 0 || index > static_cast<int>(container.size())) {
        throw std::out_of_range(
            "Index is out of range in GenericDeleteCommand::undo");
      }

      if (!removedItem) {
        throw std::runtime_error(
            "No item to restore in GenericDeleteCommand::undo");
      }

      // Put the item back at its original index.
      container.insert(container.begin() + index, std::move(removedItem));
    } catch (const std::exception &e) {
      std::cerr << "Error in GenericDeleteCommand::undo: " << e.what()
                << std::endl;
      throw; // Re-throw the exception
    }
  }

private:
  std::vector<std::unique_ptr<T>> &container;
  int index;
  std::unique_ptr<T> removedItem;
};
