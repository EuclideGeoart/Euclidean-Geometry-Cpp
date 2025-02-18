// GenericDeleteCommand.h
#pragma once
#include "Command.h"
#include <vector>
#include <memory>
#include <utility>
#include <stdexcept>

// This templated command works on any type T stored in a container (vector of unique_ptr<T>)
template <typename T>
class GenericDeleteCommand : public Command {
public:
    GenericDeleteCommand(std::vector<std::unique_ptr<T>>& container, int index)
        : container(container), index(index) {
        if (index < 0 || index >= static_cast<int>(container.size())) {
            throw std::out_of_range("Index is out of range in GenericDeleteCommand constructor");
        }
    }

    void execute() override {
        // Verify index before use
        if (index < 0 || index >= static_cast<int>(container.size())) {
            return; // or throw an exception
        }
        // Save the item (via move) so that we can restore it later.
        removedItem = std::move(container[index]);
        container.erase(container.begin() + index);
    }

    void undo() override {
        // Check index against container size for robustness.
        if (index < 0 || index > static_cast<int>(container.size())) {
            return; // or throw an exception
        }
        // Put the item back at its original index.
        container.insert(container.begin() + index, std::move(removedItem));
    }

private:
    std::vector<std::unique_ptr<T>>& container;
    int index;
    std::unique_ptr<T> removedItem;
};
