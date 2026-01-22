#ifndef CGAL_EXCEPTIONS_H
#define CGAL_EXCEPTIONS_H

// Remove any potential circular includes from here
#include <stdexcept>
#include <string>

// Base class for all CGAL-related exceptions
class CGALException : public std::runtime_error {
public:
  explicit CGALException(const std::string &msg) : std::runtime_error(msg) {}
};

// Exception for GMP-related errors
class GMP_Exception : public CGALException {
private:
  std::string m_reason;
  std::string m_expr;
  std::string m_file;

public:
  GMP_Exception(const std::string &reason, const std::string &expr,
                const std::string &file)
      : CGALException("GMP error: " + reason + " in expression: " + expr +
                      " at: " + file),
        m_reason(reason), m_expr(expr), m_file(file) {}

  const std::string &reason() const { return m_reason; }
  const std::string &expression() const { return m_expr; }
  const std::string &file() const { return m_file; }
};

// Exception for geometric inconsistencies
class GeometricInconsistencyException : public CGALException {
public:
  explicit GeometricInconsistencyException(const std::string &msg)
      : CGALException("Geometric inconsistency: " + msg) {}
};

// Exception for invalid geometric operations
class InvalidGeometricOperationException : public CGALException {
public:
  explicit InvalidGeometricOperationException(const std::string &msg)
      : CGALException("Invalid geometric operation: " + msg) {}
};

#endif // CGAL_EXCEPTIONS_H
