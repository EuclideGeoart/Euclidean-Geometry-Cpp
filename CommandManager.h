#pragma once
#include "Command.h"
#include <stack>
#include <memory>

class CommandManager {
public:
    void executeCommand(std::unique_ptr<Command> command) {
        command->execute();
        undoStack.push(std::move(command));
        // Clear the redo stack after a new action.
        while (!redoStack.empty())
            redoStack.pop();
    }

    void undo() {
        if (undoStack.empty())
            return;
        std::unique_ptr<Command> command = std::move(undoStack.top());
        undoStack.pop();
        command->undo();
        redoStack.push(std::move(command));
    }

    void redo() {
        if (redoStack.empty())
            return;
        std::unique_ptr<Command> command = std::move(redoStack.top());
        redoStack.pop();
        command->execute();
        undoStack.push(std::move(command));
    }
private:
    std::stack<std::unique_ptr<Command>> undoStack;
    std::stack<std::unique_ptr<Command>> redoStack;
};
