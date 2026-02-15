#include "TextEditorDialog.h"
#include "GeometryEditor.h"
#include "LatexRenderer.h"
#include "Constants.h"
#include "TextLabel.h"

#include <imgui.h>
#include <imgui-SFML.h>

#include <cstring>
#include <algorithm>
#include <cstdint>

TextEditorDialog::TextEditorDialog()
    : m_isOpen(false),
      m_isLatexMode(false),
      m_confirmed(false),
      m_textChanged(false),
      m_previewValid(false),
      m_previewScale(1.0f),
      m_fontSize(18.0f),
      m_textColor(sf::Color::Black) {
    
    std::memset(m_textBuffer, 0, TEXT_BUFFER_SIZE);
    
    if (!m_previewTexture.create(600, 300)) {
        // Handle error
    }
    
    initializeSymbols();
}

TextEditorDialog::~TextEditorDialog() {
}

void TextEditorDialog::initializeSymbols() {
    // Greek letters
    m_symbols.push_back({"α", "\\alpha", "Greek"});
    m_symbols.push_back({"β", "\\beta", "Greek"});
    m_symbols.push_back({"γ", "\\gamma", "Greek"});
    m_symbols.push_back({"δ", "\\delta", "Greek"});
    m_symbols.push_back({"ε", "\\epsilon", "Greek"});
    m_symbols.push_back({"θ", "\\theta", "Greek"});
    m_symbols.push_back({"λ", "\\lambda", "Greek"});
    m_symbols.push_back({"μ", "\\mu", "Greek"});
    m_symbols.push_back({"π", "\\pi", "Greek"});
    m_symbols.push_back({"ρ", "\\rho", "Greek"});
    m_symbols.push_back({"σ", "\\sigma", "Greek"});
    m_symbols.push_back({"φ", "\\phi", "Greek"});
    m_symbols.push_back({"ω", "\\omega", "Greek"});
    m_symbols.push_back({"Δ", "\\Delta", "Greek"});
    m_symbols.push_back({"Σ", "\\Sigma", "Greek"});
    m_symbols.push_back({"Ω", "\\Omega", "Greek"});
    
    // Operators
    m_symbols.push_back({"≠", "\\neq", "Symbols"});
    m_symbols.push_back({"≤", "\\leq", "Symbols"});
    m_symbols.push_back({"≥", "\\geq", "Symbols"});
    m_symbols.push_back({"≈", "\\approx", "Symbols"});
    m_symbols.push_back({"±", "\\pm", "Symbols"});
    m_symbols.push_back({"∓", "\\mp", "Symbols"});
    m_symbols.push_back({"×", "\\times", "Symbols"});
    m_symbols.push_back({"÷", "\\div", "Symbols"});
    m_symbols.push_back({"∞", "\\infty", "Symbols"});
    m_symbols.push_back({"∫", "\\int", "Symbols"});
    m_symbols.push_back({"∑", "\\sum", "Symbols"});
    m_symbols.push_back({"√", "\\sqrt{}", "Symbols"});
    
    // Geometry
    m_symbols.push_back({"°", "^\\circ", "Geometry"});
    m_symbols.push_back({"∠", "\\angle", "Geometry"});
    m_symbols.push_back({"△", "\\triangle", "Geometry"});
    m_symbols.push_back({"⊥", "\\perp", "Geometry"});
    m_symbols.push_back({"∥", "\\parallel", "Geometry"});
    m_symbols.push_back({"≅", "\\cong", "Geometry"});
    m_symbols.push_back({"∼", "\\sim", "Geometry"});
    m_symbols.push_back({"⊙", "\\odot", "Geometry"});
    
    // Math structures
    m_symbols.push_back({"a/b", "\\frac{a}{b}", "Math"});
    m_symbols.push_back({"x²", "x^{2}", "Math"});
    m_symbols.push_back({"x₂", "x_{2}", "Math"});
    m_symbols.push_back({"√x", "\\sqrt{x}", "Math"});
    m_symbols.push_back({"ⁿ√x", "\\sqrt[n]{x}", "Math"});
    m_symbols.push_back({"Σ", "\\sum_{i=1}^{n}", "Math"});
    m_symbols.push_back({"∫", "\\int_{a}^{b}", "Math"});
}

void TextEditorDialog::open(const std::string& initialText, bool isLatex, float fontSize) {
    m_isOpen = true;
    m_confirmed = false;
    m_currentText = initialText;
    m_isLatexMode = isLatex;
    m_fontSize = fontSize;
    m_textChanged = true;
    
    // Copy to buffer
    std::strncpy(m_textBuffer, initialText.c_str(), TEXT_BUFFER_SIZE - 1);
    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
    
    m_previewValid = false;
}

void TextEditorDialog::close() {
    m_isOpen = false;
}

void TextEditorDialog::updatePreview() {
    if (!m_textChanged) return;
    m_textChanged = false;
    
    LatexRenderer::initDPI();

    // 1. Calculate Scaling Logic
    float ratio = LatexRenderer::VISUAL_SCALE / 2.5f;
    float scale = LatexRenderer::HD_INVERSE * ratio;
    
    // 2. Prepare the Content (Measure First!)
    // We need to know the size BEFORE we create the background texture
    sf::Sprite spriteToDraw;
    sf::Text textToDraw;
    bool useSprite = false;
    
    float displayW = 0.0f;
    float displayH = 0.0f;

    if (m_currentText.empty()) {
        // Empty case
    }
    else if (m_isLatexMode) {
        // --- LaTeX Mode ---
        auto tex = LatexRenderer::RenderLatex(m_currentText, m_fontSize, 0, m_textColor);
        if (tex) {
            spriteToDraw.setTexture(*tex);
            spriteToDraw.setScale(scale, scale);
            
            displayW = tex->getSize().x * scale;
            displayH = tex->getSize().y * scale;
            useSprite = true;
        }
    } 
    else {
        // --- Normal Text Mode ---
        const sf::Font* font = LatexRenderer::getFont();
        if (font) {
            textToDraw.setFont(*font);
            textToDraw.setString(m_currentText);
            
            float hdFontSize = m_fontSize * LatexRenderer::HD_FACTOR * LatexRenderer::VISUAL_SCALE;
            textToDraw.setCharacterSize(static_cast<unsigned int>(std::round(hdFontSize)));
            textToDraw.setFillColor(m_textColor);
            textToDraw.setScale(scale, scale);
            
            sf::FloatRect bounds = textToDraw.getLocalBounds();
            displayW = bounds.width * scale;
            displayH = bounds.height * scale;
        }
    }

    // 3. Resize the Background Texture to Fit Content
    // Minimum size: 600x300, but grow if content is larger (+ padding)
    unsigned int requiredW = std::max(600u, static_cast<unsigned int>(displayW + 50.0f));
    unsigned int requiredH = std::max(300u, static_cast<unsigned int>(displayH + 50.0f));

    if (m_previewTexture.getSize().x != requiredW || m_previewTexture.getSize().y != requiredH) {
        m_previewTexture.create(requiredW, requiredH);
    }
    
    // 4. Draw
    m_previewTexture.clear(sf::Color::White);
    
    if (useSprite) {
        // Center/Position LaTeX
        spriteToDraw.setPosition(20.0f, 20.0f); // 20px padding
        m_previewTexture.draw(spriteToDraw);
    } else if (!m_currentText.empty()) {
        // Position Text (accounting for glyph offsets)
        sf::FloatRect bounds = textToDraw.getLocalBounds();
        textToDraw.setPosition(20.0f - (bounds.left * scale), 20.0f - (bounds.top * scale));
        m_previewTexture.draw(textToDraw);
    }
    
    m_previewTexture.display();
    m_previewValid = true;
}

void TextEditorDialog::renderSymbolTabs() {
    if (ImGui::BeginTabBar("SymbolTabs")) {
        float ratio = LatexRenderer::VISUAL_SCALE / 2.5f;
        float windowWidth = ImGui::GetContentRegionAvail().x;
        
        // Symbols tab
        if (ImGui::BeginTabItem("Symbols")) {
            ImGui::BeginChild("SymbolsScroll", ImVec2(0, 120 * ratio), true);
            
            float buttonWidth = 50.0f * ratio;
            float buttonHeight = 30.0f * ratio;
            float spacing = ImGui::GetStyle().ItemSpacing.x;
            
            float currentX = 0.0f;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Symbols") continue;
                
                if (currentX + buttonWidth > windowWidth && currentX > 0.0f) {
                    currentX = 0.0f;
                } else if (currentX > 0.0f) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(buttonWidth, buttonHeight))) {
                    // Insert symbol at cursor position
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                currentX += buttonWidth + spacing;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Greek tab
        if (ImGui::BeginTabItem("Greek")) {
            ImGui::BeginChild("GreekScroll", ImVec2(0, 120 * ratio), true);
            
            float buttonWidth = 50.0f * ratio;
            float buttonHeight = 30.0f * ratio;
            float spacing = ImGui::GetStyle().ItemSpacing.x;
            float currentX = 0.0f;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Greek") continue;
                
                if (currentX + buttonWidth > windowWidth && currentX > 0.0f) {
                    currentX = 0.0f;
                } else if (currentX > 0.0f) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(buttonWidth, buttonHeight))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                currentX += buttonWidth + spacing;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Geometry tab
        if (ImGui::BeginTabItem("Geometry")) {
            ImGui::BeginChild("GeometryScroll", ImVec2(0, 120 * ratio), true);
            
            float buttonWidth = 50.0f * ratio;
            float buttonHeight = 30.0f * ratio;
            float spacing = ImGui::GetStyle().ItemSpacing.x;
            float currentX = 0.0f;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Geometry") continue;
                
                if (currentX + buttonWidth > windowWidth && currentX > 0.0f) {
                    currentX = 0.0f;
                } else if (currentX > 0.0f) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(buttonWidth, buttonHeight))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                currentX += buttonWidth + spacing;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Math tab
        if (ImGui::BeginTabItem("Math")) {
            ImGui::BeginChild("MathScroll", ImVec2(0, 120 * ratio), true);
            
            float buttonWidth = 70.0f * ratio;
            float buttonHeight = 30.0f * ratio;
            float spacing = ImGui::GetStyle().ItemSpacing.x;
            float currentX = 0.0f;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Math") continue;
                
                if (currentX + buttonWidth > windowWidth && currentX > 0.0f) {
                    currentX = 0.0f;
                } else if (currentX > 0.0f) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(buttonWidth, buttonHeight))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                currentX += buttonWidth + spacing;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
}

void TextEditorDialog::renderPreviewArea() {
    ImGui::Text("Preview:");
    
    // 1. Calculate the window height
    float ratio = LatexRenderer::VISUAL_SCALE / 2.5f;
    
    // 2. Enable Horizontal Scrollbar and vertical scrollbar
    ImGui::BeginChild("PreviewArea", ImVec2(0, 200 * ratio), true, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
    
    if (m_previewValid && m_previewTexture.getTexture().getNativeHandle() != 0) {
        const sf::Texture& tex = m_previewTexture.getTexture();
        sf::Vector2u texSize = tex.getSize();
        
        // --- SIZE ADJUSTMENT ---
        // The texture is HD (High-Res), so 1.0f looks huge.
        // 0.45f roughly compensates for the HD factor to match the input text size.
        float viewScale = 0.45f; 
        
        ImVec2 displaySize(texSize.x * viewScale, texSize.y * viewScale);
        
        // UV coordinates to fix any flipping issues
        ImVec2 uv0(0, 1);
        ImVec2 uv1(1, 0);
        
        // Render the image
        ImGui::Image((void*)(intptr_t)tex.getNativeHandle(), displaySize, uv0, uv1);
    } else {
        ImGui::TextDisabled("No preview available");
    }
    
    ImGui::EndChild();
}

void TextEditorDialog::render(sf::RenderWindow& window, GeometryEditor& editor) {
    if (!m_isOpen) return;
    
    float ratio = LatexRenderer::VISUAL_SCALE / 2.5f;
    ImGui::SetNextWindowSize(ImVec2(650 * ratio, 600 * ratio), ImGuiCond_FirstUseEver);
    
    if (ImGui::Begin("Text Editor", &m_isOpen, ImGuiWindowFlags_NoCollapse)) {
        // Top section: Text editor
        ImGui::Text("Edit Text:");
        
        ImGuiInputTextFlags flags = ImGuiInputTextFlags_AllowTabInput;
        if (ImGui::InputTextMultiline("##textinput", m_textBuffer, TEXT_BUFFER_SIZE, 
                                       ImVec2(-1, 150 * ratio), flags)) {
            m_currentText = std::string(m_textBuffer);
            m_textChanged = true;
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        // Middle section: LaTeX mode and symbols
        if (ImGui::Checkbox("LaTeX formula", &m_isLatexMode)) {
            m_textChanged = true;
        }
        
        ImGui::SameLine();
        ImGui::Text("Font Size:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100 * ratio);
        if (ImGui::DragFloat("##fontsize", &m_fontSize, 0.5f, 6.0f, 72.0f, "%.1f")) {
            m_textChanged = true;
        }
        
        ImGui::Spacing();
        
        if (m_isLatexMode) {
            renderSymbolTabs();
        }
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        // Update preview if needed
        updatePreview();
        
        // Bottom section: Preview
        renderPreviewArea();
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        // Footer: OK/Cancel buttons
        float buttonWidth = 100.0f * ratio;
        float spacing = ImGui::GetStyle().ItemSpacing.x;
        float totalWidth = buttonWidth * 2 + spacing;
        float offsetX = (ImGui::GetContentRegionAvail().x - totalWidth) * 0.5f;
        
        if (offsetX > 0) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offsetX);
        
        if (ImGui::Button("OK", ImVec2(buttonWidth, 0))) {
            m_confirmed = true;
            m_resultText = m_currentText;
            m_isOpen = false;
        }
        
        ImGui::SameLine();
        
        if (ImGui::Button("Cancel", ImVec2(buttonWidth, 0))) {
            m_confirmed = false;
            m_isOpen = false;

            editor.textEditingLabel = nullptr;
            editor.isEditingExistingText = false;
        }
    }
    ImGui::End();
    
    // If dialog was closed via 'X', treat as cancel
    if (!m_isOpen && !m_confirmed) {
        m_confirmed = false;
        editor.textEditingLabel = nullptr;
        editor.isEditingExistingText = false;
    }
}
