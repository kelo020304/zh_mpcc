#!/bin/bash
# ============================================================================
# HPIPM/BLASFEO Installation Checker
#
# This script checks if HPIPM and BLASFEO are correctly installed
#
# Usage:
#   bash check_hpipm.sh
#
# Author: Local Planner Project
# Date: 2026-02
# ============================================================================

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "======================================"
echo "HPIPM/BLASFEO Installation Checker"
echo "======================================"
echo ""

# Function to check file existence
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Missing: $1"
        return 1
    fi
}

# Function to check directory existence
check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Missing: $1"
        return 1
    fi
}

INSTALL_PREFIX=/usr/local
ERRORS=0

echo "Checking installation in: $INSTALL_PREFIX"
echo ""

# Check BLASFEO
echo "===== BLASFEO ====="
check_file "$INSTALL_PREFIX/lib/libblasfeo.a" || ((ERRORS++))
check_dir "$INSTALL_PREFIX/include/blasfeo" || ((ERRORS++))
check_file "$INSTALL_PREFIX/include/blasfeo/blasfeo_d_aux_ext_dep.h" || ((ERRORS++))
echo ""

# Check HPIPM
echo "===== HPIPM ====="
check_file "$INSTALL_PREFIX/lib/libhpipm.a" || ((ERRORS++))
check_dir "$INSTALL_PREFIX/include/hpipm" || ((ERRORS++))
check_file "$INSTALL_PREFIX/include/hpipm/hpipm_d_ocp_qp.h" || ((ERRORS++))
echo ""

# Try to compile a test program
echo "===== Compilation Test ====="
TEST_FILE=$(mktemp /tmp/test_hpipm_XXXXXX.cpp)
cat > "$TEST_FILE" << 'EOF'
#include <hpipm_d_ocp_qp.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <iostream>

int main() {
    std::cout << "Headers found!" << std::endl;
    return 0;
}
EOF

if g++ "$TEST_FILE" -o /tmp/test_hpipm_compile \
    -I$INSTALL_PREFIX/include \
    -L$INSTALL_PREFIX/lib \
    -lhpipm -lblasfeo -lm 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Compilation test passed"
    rm -f /tmp/test_hpipm_compile "$TEST_FILE"
else
    echo -e "${RED}✗${NC} Compilation test failed"
    echo "    Headers or libraries may be misconfigured"
    ((ERRORS++))
    rm -f "$TEST_FILE"
fi
echo ""

# Check for alternative installation locations
if [ $ERRORS -gt 0 ]; then
    echo "===== Checking Alternative Locations ====="

    # Check ~/.local
    if [ -f "$HOME/.local/hpipm/lib/libhpipm.a" ] || [ -f "$HOME/.local/lib/libhpipm.a" ]; then
        echo -e "${YELLOW}!${NC} Found HPIPM in ~/.local"
        echo "    You may need to set HPIPM_ROOT=$HOME/.local/hpipm"
    fi

    if [ -f "$HOME/.local/blasfeo/lib/libblasfeo.a" ] || [ -f "$HOME/.local/lib/libblasfeo.a" ]; then
        echo -e "${YELLOW}!${NC} Found BLASFEO in ~/.local"
        echo "    You may need to set BLASFEO_ROOT=$HOME/.local/blasfeo"
    fi

    # Check /usr/lib
    if [ -f "/usr/lib/libhpipm.a" ] || [ -f "/usr/lib/x86_64-linux-gnu/libhpipm.a" ]; then
        echo -e "${YELLOW}!${NC} Found HPIPM in /usr/lib"
    fi

    if [ -f "/usr/lib/libblasfeo.a" ] || [ -f "/usr/lib/x86_64-linux-gnu/libblasfeo.a" ]; then
        echo -e "${YELLOW}!${NC} Found BLASFEO in /usr/lib"
    fi
    echo ""
fi

# Summary
echo "======================================"
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed!${NC}"
    echo "HPIPM and BLASFEO are correctly installed."
    echo ""
    echo "You can now compile local_planner:"
    echo "  cd /path/to/catkin_ws"
    echo "  catkin_make"
    exit 0
else
    echo -e "${RED}✗ $ERRORS error(s) found${NC}"
    echo ""
    echo "Installation is incomplete or incorrect."
    echo ""
    echo "To fix this, run:"
    echo "  cd /path/to/local_planner"
    echo "  bash scripts/install_hpipm.sh"
    echo ""
    echo "Or follow the manual installation steps in README.md"
    exit 1
fi
