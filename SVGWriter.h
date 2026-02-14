#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <fstream>

/**
 * @brief Professional, Header-Only SVG Writer Class
 * 
 * DESIGN GOALS:
 * 1. Clean, Fluent API (Chaining)
 * 2. Automatic XML Escaping (Basic)
 * 3. Minimal Dependencies (Standard Library Only)
 * 4. Precision Control
 */
class SVGWriter {
public:
    // --- Styles ---
    struct Style {
        std::string fill = "none";
        std::string stroke = "black";
        double strokeWidth = 1.0;
        double opacity = 1.0;
        std::string strokeLineCap = "round";
        std::string strokeLineJoin = "round";
        std::vector<double> strokeDashArray;
        double fontSize = 12.0;
        std::string fontFamily = "Arial, sans-serif";
        std::string textAnchor = "start"; // start, middle, end
        std::string dominantBaseline = "auto"; // auto, middle, central

        std::string toString() const {
            std::stringstream ss;
            ss << "fill=\"" << fill << "\" ";
            ss << "stroke=\"" << stroke << "\" ";
            ss << "stroke-width=\"" << strokeWidth << "\" ";
            if (opacity != 1.0) ss << "opacity=\"" << opacity << "\" ";
            ss << "stroke-linecap=\"" << strokeLineCap << "\" ";
            ss << "stroke-linejoin=\"" << strokeLineJoin << "\" ";
            if (!strokeDashArray.empty()) {
                ss << "stroke-dasharray=\"";
                for (size_t i = 0; i < strokeDashArray.size(); ++i) {
                    ss << strokeDashArray[i] << (i < strokeDashArray.size() - 1 ? "," : "");
                }
                ss << "\" ";
            }
            ss << "font-size=\"" << fontSize << "\" ";
            ss << "font-family=\"" << fontFamily << "\" ";
            ss << "text-anchor=\"" << textAnchor << "\" ";
            ss << "dominant-baseline=\"" << dominantBaseline << "\" ";
            return ss.str();
        }
    };

    // --- Constructor ---
    SVGWriter(double width, double height) 
        : m_width(width), m_height(height), m_minX(0), m_minY(0) {}

    // Set custom bounds (ViewBox)
    void setViewBox(double minX, double minY, double width, double height) {
        m_minX = minX;
        m_minY = minY;
        m_width = width;
        m_height = height;
    }

    // Group helpers (for transforms like Y-flip)
    void beginGroup(const std::string& transform = std::string()) {
        m_body << "<g";
        if (!transform.empty()) {
            m_body << " transform=\"" << transform << "\"";
        }
        m_body << ">\n";
    }

    void endGroup() {
        m_body << "</g>\n";
    }

    // --- Core Drawing Methods ---

    void drawLine(double x1, double y1, double x2, double y2, const Style& style) {
        m_body << "<line x1=\"" << x1 << "\" y1=\"" << y1 << "\" "
               << "x2=\"" << x2 << "\" y2=\"" << y2 << "\" "
               << style.toString() << "/>\n";
    }

    void drawCircle(double cx, double cy, double r, const Style& style) {
        m_body << "<circle cx=\"" << cx << "\" cy=\"" << cy << "\" r=\"" << r << "\" "
               << style.toString() << "/>\n";
    }

    void drawRect(double x, double y, double width, double height, const Style& style) {
        m_body << "<rect x=\"" << x << "\" y=\"" << y << "\" "
               << "width=\"" << width << "\" height=\"" << height << "\" "
               << style.toString() << "/>\n";
    }

    void drawPolygon(const std::vector<std::pair<double, double>>& points, const Style& style) {
        if (points.empty()) return;
        m_body << "<polygon points=\"";
        for (size_t i = 0; i < points.size(); ++i) {
            m_body << points[i].first << "," << points[i].second << (i < points.size() - 1 ? " " : "");
        }
        m_body << "\" " << style.toString() << "/>\n";
    }

    void drawText(double x, double y, const std::string& content, const Style& style) {
        m_body << "<text x=\"" << x << "\" y=\"" << y << "\" "
               << style.toString() << ">" << escapeXML(content) << "</text>\n";
    }

    void drawPath(const std::string& pathData, const Style& style) {
        m_body << "<path d=\"" << pathData << "\" "
               << style.toString() << "/>\n";
    }

    void addCustom(const std::string& customXML) {
        m_body << customXML << "\n";
    }

    // --- File Output ---
    void setOutputSize(double w, double h) {
        m_outputWidth = w;
        m_outputHeight = h;
    }

    bool save(const std::string& filepath) {
        std::ofstream file(filepath);
        if (!file.is_open()) return false;

        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
        file << "viewBox=\"" << m_minX << " " << m_minY << " " << m_width << " " << m_height << "\" ";
        file << "width=\"" << m_outputWidth << "\" height=\"" << m_outputHeight << "\">\n"; 
        
        file << std::fixed << std::setprecision(4);
        file << m_body.str();
        
        file << "</svg>\n";
        return true;
    }

private:
    double m_width, m_height, m_minX, m_minY;
    double m_outputWidth = 800.0;
    double m_outputHeight = 800.0;
    std::stringstream m_body;

    std::string escapeXML(const std::string& data) {
        std::string buffer;
        buffer.reserve(data.size());
        for (char c : data) {
            switch (c) {
                case '&': buffer.append("&amp;"); break;
                case '\"': buffer.append("&quot;"); break;
                case '\'': buffer.append("&apos;"); break;
                case '<': buffer.append("&lt;"); break;
                case '>': buffer.append("&gt;"); break;
                default: buffer.push_back(c); break;
            }
        }
        return buffer;
    }
};
