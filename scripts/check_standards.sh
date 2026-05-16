#!/bin/bash
# NORM-Nav coding standards checker
#
# Purpose: Quick audit against project conventions.
# Author: Wang Junhui <wjh_9696@163.com>

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║        NORM-Nav coding standards checker            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

check_passed() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    PASSED_CHECKS=$((PASSED_CHECKS + 1))
    echo -e "${GREEN}✓${NC} $1"
}

check_failed() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    FAILED_CHECKS=$((FAILED_CHECKS + 1))
    echo -e "${RED}✗${NC} $1"
    echo -e "  ${YELLOW}→${NC} $2"
}

check_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

echo -e "${BLUE}[1/7] Package naming...${NC}"

BAD_PACKAGES=$(find src/ -maxdepth 2 -name "package.xml" -exec dirname {} \; | \
    grep -v "FAST_LIO\|livox\|pointcloud_to_laserscan\|linefit_ground" | \
    xargs -I {} basename {} | \
    grep -v "^navibot_\|^src$\|^localization$\|^navigation$\|^perception$\|^simulation$\|^sensors$" || true)

if [ -z "$BAD_PACKAGES" ]; then
    check_passed "All first-party packages use the navibot_ prefix"
else
    check_failed "Packages violate naming rules" "$BAD_PACKAGES"
fi

echo -e "\n${BLUE}[2/7] package.xml metadata...${NC}"

NAVIBOT_PACKAGES=$(find src/ -path "*/navibot_*/package.xml")

TODO_COUNT=0
VERSION_ISSUES=0
LICENSE_ISSUES=0

for pkg in $NAVIBOT_PACKAGES; do
    if grep -q "TODO" "$pkg"; then
        TODO_COUNT=$((TODO_COUNT + 1))
    fi

    VERSION=$(grep -oP '<version>\K[^<]+' "$pkg" | head -1)
    if [ "$VERSION" != "1.0.0" ]; then
        VERSION_ISSUES=$((VERSION_ISSUES + 1))
    fi

    LICENSE=$(grep -oP '<license>\K[^<]+' "$pkg" | head -1)
    if [ "$LICENSE" != "MIT" ]; then
        LICENSE_ISSUES=$((LICENSE_ISSUES + 1))
    fi
done

if [ $TODO_COUNT -eq 0 ]; then
    check_passed "No TODO placeholders in package.xml"
else
    check_failed "$TODO_COUNT package.xml file(s) contain TODO" "Remove TODOs and fill real metadata"
fi

if [ $VERSION_ISSUES -eq 0 ]; then
    check_passed "Version pinned to 1.0.0"
else
    check_failed "$VERSION_ISSUES version mismatch(es)" "Standardize on 1.0.0"
fi

if [ $LICENSE_ISSUES -eq 0 ]; then
    check_passed "License set to MIT"
else
    check_failed "$LICENSE_ISSUES license mismatch(es)" "Standardize on MIT"
fi

echo -e "\n${BLUE}[3/7] Python conventions...${NC}"

PYTHON_FILES=$(find src/navibot_* -name "*.py" -not -path "*/build/*" -not -path "*/install/*" -not -path "*/__pycache__/*" 2>/dev/null || true)

if [ -n "$PYTHON_FILES" ]; then
    NO_HEADER=0
    for file in $PYTHON_FILES; do
        if ! grep -q "Author: Wang Junhui" "$file"; then
            NO_HEADER=$((NO_HEADER + 1))
        fi
    done

    if [ $NO_HEADER -eq 0 ]; then
        check_passed "Python files include the standard header"
    else
        check_failed "$NO_HEADER Python file(s) missing header" "Add Author: Wang Junhui <wjh_9696@163.com>"
    fi

    PLACEHOLDER_EMAIL=$(grep -r "wjh@todo\|fyt@todo" src/navibot_* --include="*.py" 2>/dev/null | wc -l)
    if [ "$PLACEHOLDER_EMAIL" -eq 0 ]; then
        check_passed "No placeholder emails"
    else
        check_failed "$PLACEHOLDER_EMAIL placeholder email hit(s)" "Replace with real addresses"
    fi
else
    check_warning "No Python files found under navibot_*"
fi

echo -e "\n${BLUE}[4/7] C++ conventions...${NC}"

CPP_FILES=$(find src/navibot_* -name "*.cpp" -o -name "*.hpp" -o -name "*.h" 2>/dev/null || true)

if [ -n "$CPP_FILES" ]; then
    USING_NS=$(grep -r "using namespace std" src/navibot_* --include="*.cpp" --include="*.hpp" --include="*.h" 2>/dev/null | wc -l)
    if [ "$USING_NS" -eq 0 ]; then
        check_passed "No using namespace std"
    else
        check_failed "$USING_NS using namespace std occurrence(s)" "Prefer explicit std::"
    fi

    NO_DOXYGEN=0
    for file in $CPP_FILES; do
        if ! grep -q "@author\|@brief" "$file"; then
            NO_DOXYGEN=$((NO_DOXYGEN + 1))
        fi
    done

    if [ $NO_DOXYGEN -eq 0 ]; then
        check_passed "C++ files include Doxygen tags"
    else
        check_failed "$NO_DOXYGEN file(s) missing Doxygen" "Add @author, @brief, etc."
    fi
else
    check_warning "No C++ files found under navibot_*"
fi

echo -e "\n${BLUE}[5/7] Launch files...${NC}"

LAUNCH_FILES=$(find src/navibot_* -name "*.launch.py" 2>/dev/null || true)

if [ -n "$LAUNCH_FILES" ]; then
    NO_USAGE=0
    NO_TYPE_HINT=0

    for file in $LAUNCH_FILES; do
        if ! grep -q "Usage:" "$file"; then
            NO_USAGE=$((NO_USAGE + 1))
        fi
        if ! grep -q "LaunchDescription:" "$file"; then
            NO_TYPE_HINT=$((NO_TYPE_HINT + 1))
        fi
    done

    if [ $NO_USAGE -eq 0 ]; then
        check_passed "Launch files document Usage"
    else
        check_failed "$NO_USAGE launch file(s) missing Usage" "Add a Usage: section"
    fi

    if [ $NO_TYPE_HINT -eq 0 ]; then
        check_passed "Launch files annotate LaunchDescription"
    else
        check_failed "$NO_TYPE_HINT launch file(s) missing return type" "Add -> LaunchDescription"
    fi
else
    check_warning "No launch files found"
fi

echo -e "\n${BLUE}[6/7] YAML configs...${NC}"

YAML_FILES=$(find src/navibot_* -name "*.yaml" 2>/dev/null || true)

if [ -n "$YAML_FILES" ]; then
    YAML_WITH_COMMENTS=0
    TOTAL_YAML=0

    for file in $YAML_FILES; do
        TOTAL_YAML=$((TOTAL_YAML + 1))
        if grep -q "^#" "$file"; then
            YAML_WITH_COMMENTS=$((YAML_WITH_COMMENTS + 1))
        fi
    done

    if [ $YAML_WITH_COMMENTS -eq $TOTAL_YAML ]; then
        check_passed "YAML files start with at least one comment line"
    else
        MISSING=$((TOTAL_YAML - YAML_WITH_COMMENTS))
        check_failed "$MISSING YAML file(s) lack leading comments" "Document parameters with # comments"
    fi
else
    check_warning "No YAML files found"
fi

echo -e "\n${BLUE}[7/7] Documentation...${NC}"

docs=("CODING_STANDARDS.md" "AI_PROMPT_TEMPLATE.md" "REFACTORING_COMPLETE.md" ".cursorrules")
MISSING_DOCS=0

for doc in "${docs[@]}"; do
    if [ ! -f "$doc" ]; then
        MISSING_DOCS=$((MISSING_DOCS + 1))
    fi
done

if [ $MISSING_DOCS -eq 0 ]; then
    check_passed "Expected policy docs are present"
else
    check_failed "$MISSING_DOCS expected doc(s) missing" "Add the referenced standards files"
fi

NAVIBOT_DIRS=$(find src/ -maxdepth 2 -type d -name "navibot_*")
NO_README=0

for dir in $NAVIBOT_DIRS; do
    if [ ! -f "$dir/README.md" ]; then
        NO_README=$((NO_README + 1))
    fi
done

if [ $NO_README -eq 0 ]; then
    check_passed "Every navibot_* package has README.md"
else
    check_warning "$NO_README package(s) missing README.md (recommended)"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}Summary${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "Checks run: $TOTAL_CHECKS"
echo -e "${GREEN}Passed: $PASSED_CHECKS${NC}"
echo -e "${RED}Failed: $FAILED_CHECKS${NC}"
echo ""

if [ $FAILED_CHECKS -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed.${NC}"
    echo -e "${GREEN}✓ Repository matches NORM-Nav coding standards.${NC}"
    exit 0
else
    echo -e "${RED}✗ $FAILED_CHECKS check(s) failed.${NC}"
    echo -e "${YELLOW}See CODING_STANDARDS.md for remediation.${NC}"
    exit 1
fi
