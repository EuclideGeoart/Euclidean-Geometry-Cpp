#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

class GeometryEditor;

struct SymbolButton {
    std::string label;
    std::string latexCode;
    std::string category;
};

class TextEditorDialog {
public:
    TextEditorDialog();
    ~TextEditorDialog();

    void open(const std::string& initialText = "", bool isLatex = false, float fontSize = 18.0f);
    void close();
    void render(sf::RenderWindow& window, class GeometryEditor& editor);
    
    bool isDialogOpen() const { return m_isOpen; }
    const std::string& getResultText() const { return m_resultText; }
    bool wasConfirmed() const { return m_confirmed; }
    bool isLatexResult() const { return m_isLatexMode; }
    float getFontSize() const { return m_fontSize; }

private:
    void updatePreview();
    void renderSymbolTabs();
    void renderPreviewArea();
    void initializeSymbols();

    // State
    bool m_isOpen;
    bool m_isLatexMode;
    bool m_confirmed;
    std::string m_currentText;
    std::string m_resultText;
    bool m_textChanged;
    
    // Preview rendering
    sf::RenderTexture m_previewTexture;
    bool m_previewValid;
    float m_previewScale;
    
    // Symbol categories
    std::vector<SymbolButton> m_symbols;
    std::string m_currentCategory;
    
    // UI state
    static constexpr int TEXT_BUFFER_SIZE = 4096;
    char m_textBuffer[TEXT_BUFFER_SIZE];
    float m_fontSize;
    sf::Color m_textColor;
};
