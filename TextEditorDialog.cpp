#include "TextEditorDialog.h"
#include "GeometryEditor.h"
#include "LatexRenderer.h"
#include "Constants.h"
#include "TextLabel.h"

#include "imgui.h"
#include "imgui-SFML.h"

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
    
    m_previewTexture.clear(sf::Color::White);
    
    if (m_currentText.empty()) {
        m_previewTexture.display();
        m_previewValid = true;
        return;
    }
    
    if (m_isLatexMode) {
        // Render LaTeX with high quality
        sf::Texture* rawTex = LatexRenderer::RenderLatex(m_currentText, m_fontSize, 0.0f, m_textColor);
        std::shared_ptr<sf::Texture> texture(rawTex);
        
        if (texture) {
            sf::Sprite sprite(*texture);
            
            // Center in preview area
            sf::Vector2u texSize = texture->getSize();
            sf::Vector2u previewSize = m_previewTexture.getSize();
            
            float x = (previewSize.x - texSize.x) * 0.5f;
            float y = (previewSize.y - texSize.y) * 0.5f;
            
            sprite.setPosition(x, y);
            m_previewTexture.draw(sprite);
        }
    } else {
        // Render plain text
        const sf::Font* font = LatexRenderer::getFont();
        if (font) {
            sf::Text text;
            text.setFont(*font);
            text.setString(m_currentText);
            text.setCharacterSize(static_cast<unsigned int>(m_fontSize));
            text.setFillColor(m_textColor);
            
            // Center in preview area
            sf::FloatRect bounds = text.getLocalBounds();
            sf::Vector2u previewSize = m_previewTexture.getSize();
            
            float x = (previewSize.x - bounds.width) * 0.5f;
            float y = (previewSize.y - bounds.height) * 0.5f;
            
            text.setPosition(x, y);
            m_previewTexture.draw(text);
        }
    }
    
    m_previewTexture.display();
    m_previewValid = true;
}

void TextEditorDialog::renderSymbolTabs() {
    if (ImGui::BeginTabBar("SymbolTabs")) {
        // Symbols tab
        if (ImGui::BeginTabItem("Symbols")) {
            ImGui::BeginChild("SymbolsScroll", ImVec2(0, 120), true);
            
            int buttonsPerRow = 8;
            int count = 0;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Symbols") continue;
                
                if (count > 0 && count % buttonsPerRow != 0) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(50, 30))) {
                    // Insert symbol at cursor position
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                count++;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Greek tab
        if (ImGui::BeginTabItem("Greek")) {
            ImGui::BeginChild("GreekScroll", ImVec2(0, 120), true);
            
            int buttonsPerRow = 8;
            int count = 0;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Greek") continue;
                
                if (count > 0 && count % buttonsPerRow != 0) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(50, 30))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                count++;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Geometry tab
        if (ImGui::BeginTabItem("Geometry")) {
            ImGui::BeginChild("GeometryScroll", ImVec2(0, 120), true);
            
            int buttonsPerRow = 8;
            int count = 0;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Geometry") continue;
                
                if (count > 0 && count % buttonsPerRow != 0) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(50, 30))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                count++;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        // Math tab
        if (ImGui::BeginTabItem("Math")) {
            ImGui::BeginChild("MathScroll", ImVec2(0, 120), true);
            
            int buttonsPerRow = 6;
            int count = 0;
            
            for (const auto& sym : m_symbols) {
                if (sym.category != "Math") continue;
                
                if (count > 0 && count % buttonsPerRow != 0) {
                    ImGui::SameLine();
                }
                
                if (ImGui::Button(sym.label.c_str(), ImVec2(70, 30))) {
                    m_currentText += sym.latexCode;
                    std::strncpy(m_textBuffer, m_currentText.c_str(), TEXT_BUFFER_SIZE - 1);
                    m_textBuffer[TEXT_BUFFER_SIZE - 1] = '\0';
                    m_textChanged = true;
                }
                
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", sym.latexCode.c_str());
                }
                
                count++;
            }
            
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
}

void TextEditorDialog::renderPreviewArea() {
    ImGui::Text("Preview:");
    ImGui::BeginChild("PreviewArea", ImVec2(0, 200), true, ImGuiWindowFlags_NoScrollbar);
    
    if (m_previewValid && m_previewTexture.getTexture().getNativeHandle() != 0) {
        const sf::Texture& tex = m_previewTexture.getTexture();
        sf::Vector2u texSize = tex.getSize();
        
        // Calculate scaled size to fit width
        float availWidth = ImGui::GetContentRegionAvail().x;
        float scale = std::min(1.0f, availWidth / static_cast<float>(texSize.x));
        ImVec2 displaySize(texSize.x * scale, texSize.y * scale);
        
        // Fix flipped preview: Swap UV coordinates vertically
        ImVec2 uv0(0, 1);  // Bottom-left
        ImVec2 uv1(1, 0);  // Top-right
        
        // Mandatory Fix 3: Use Native Handle to force correct UVs
        ImGui::Image((void*)(intptr_t)tex.getNativeHandle(), displaySize, uv0, uv1);
    } else {
        ImGui::TextDisabled("No preview available");
    }
    
    ImGui::EndChild();
}

void TextEditorDialog::render(sf::RenderWindow& window, GeometryEditor& editor) {
    if (!m_isOpen) return;
    
    ImGui::SetNextWindowSize(ImVec2(650, 600), ImGuiCond_FirstUseEver);
    
    if (ImGui::Begin("Text Editor", &m_isOpen, ImGuiWindowFlags_NoCollapse)) {
        // Top section: Text editor
        ImGui::Text("Edit Text:");
        
        ImGuiInputTextFlags flags = ImGuiInputTextFlags_AllowTabInput;
        if (ImGui::InputTextMultiline("##textinput", m_textBuffer, TEXT_BUFFER_SIZE, 
                                       ImVec2(-1, 150), flags)) {
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
        ImGui::SetNextItemWidth(100);
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
        float buttonWidth = 100.0f;
        float spacing = ImGui::GetStyle().ItemSpacing.x;
        float totalWidth = buttonWidth * 2 + spacing;
        float offsetX = (ImGui::GetContentRegionAvail().x - totalWidth) * 0.5f;
        
        if (offsetX > 0) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offsetX);
        
        if (ImGui::Button("OK", ImVec2(buttonWidth, 0))) {
            m_confirmed = true;
            m_resultText = m_currentText;
            m_isOpen = false;
            
            // --- SAVE LOGIC ---
            if (editor.textEditingLabel) {
                // Determine if this is a change that warrants a new command or updating existing
                // For now, simple update
                if (editor.isEditingExistingText) {
                    // TODO: Push Undo Command for property change
                }
                
                editor.textEditingLabel->setRawContent(m_currentText, m_isLatexMode);
                editor.textEditingLabel->setFontSize(m_fontSize);
                editor.textEditingLabel->update(); // Force refresh
                
                // Clear pointer to release "lock" on editing
                editor.textEditingLabel = nullptr;
                editor.isEditingExistingText = false;
            }
        }
        
        ImGui::SameLine();
        
        if (ImGui::Button("Cancel", ImVec2(buttonWidth, 0))) {
            m_confirmed = false;
            m_isOpen = false;

            // --- CANCEL LOGIC ---
            if (!editor.isEditingExistingText && editor.textEditingLabel) {
                 // If we were creating a NEW label and cancelled, we should remove it (Undo creation)
                 // Assuming creation command was already pushed, we might need to Undo it? 
                 // Or easier: manually remove it from textLabels list if checks passed.
                 // For safety with CommandSystem, ideally we call editor.cancelLastOperation() if supported
                 // Or just let it exist as empty? No.
                 // editor.deleteObject(editor.textEditingLabel.get()); // If such method exists
                 
                 // Fallback: If "New Label", user expects it to vanish if Cancelled.
                 // Using editor to find and remove.
                 auto& labels = editor.textLabels;
                 auto it = std::find(labels.begin(), labels.end(), editor.textEditingLabel);
                 if (it != labels.end()) {
                     labels.erase(it);
                 }
                 // And remove command history if possible? 
                 // editor.commandManager.undo(); // If creation was the last command
                 editor.commandManager.undo(); 
            }
            
            editor.textEditingLabel = nullptr;
            editor.isEditingExistingText = false;
        }
    }
    ImGui::End();
    
    // If dialog was closed via 'X', treat as cancel
    if (!m_isOpen && !m_confirmed) {
        m_confirmed = false;
        // Same cancel logic
        if (!editor.isEditingExistingText && editor.textEditingLabel) {
             auto& labels = editor.textLabels;
             auto it = std::find(labels.begin(), labels.end(), editor.textEditingLabel);
             if (it != labels.end()) {
                 labels.erase(it);
             }
             editor.commandManager.undo();
        }
        editor.textEditingLabel = nullptr;
        editor.isEditingExistingText = false;
    }
}
