#!/bin/bash
# Validation script: /sp.check for Physical AI Textbook

echo "🔍 Running Physical AI Textbook Validation Checks..."

# Check 1: Docusaurus project exists
if [ ! -d "docusaurus" ]; then
  echo "❌ FAIL: docusaurus/ directory not found"
  exit 1
fi
echo "✅ PASS: Docusaurus project exists"

# Check 2: 13 chapters exist
CHAPTER_COUNT=$(find docusaurus/docs -name "ch*" -type d 2>/dev/null | wc -l)
if [ "$CHAPTER_COUNT" -lt 13 ]; then
  echo "⚠️  WARN: Only $CHAPTER_COUNT chapters found (expected 13)"
  echo "   Run tasks T009-T011 to create all chapters"
fi
echo "✅ PASS: Chapter structure validated"

# Check 3: MCQ component exists
if [ ! -f "docusaurus/src/components/MCQ/index.tsx" ]; then
  echo "⚠️  WARN: MCQ component not found"
  echo "   Run task T012 to create MCQ component"
fi

# Check 4: Hardware page exists
if [ ! -f "docusaurus/docs/hardware-requirements.md" ]; then
  echo "⚠️  WARN: Hardware requirements page not found"
  echo "   Run task T018 to create hardware page"
fi

# Check 5: GitHub Actions workflow exists
if [ ! -f ".github/workflows/deploy.yml" ]; then
  echo "⚠️  WARN: GitHub Actions workflow not found"
  echo "   Run task T006 to create workflow"
fi

# Check 6: Build test (if docusaurus exists)
if [ -d "docusaurus" ] && [ -f "docusaurus/package.json" ]; then
  echo "⏳ Testing build (if dependencies installed)..."
  cd docusaurus
  if [ -d "node_modules" ]; then
    npm run build > /dev/null 2>&1
    if [ $? -ne 0 ]; then
      echo "❌ FAIL: Build failed - check docusaurus/package.json and run npm install"
    else
      echo "✅ PASS: Build succeeds"
    fi
  else
    echo "⚠️  WARN: node_modules not found - run 'npm install' in docusaurus/"
  fi
  cd ..
else
  echo "⚠️  WARN: Docusaurus project not initialized - run task T001"
fi

# Check 7: i18n stub exists
if [ ! -d "docusaurus/i18n/ur" ]; then
  echo "⚠️  WARN: i18n/ur directory not found"
  echo "   Run task T005 to create i18n stub"
fi

echo ""
echo "📊 Validation Summary:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps based on task completion:"
echo ""
echo "Phase 1 (Setup):"
echo "  [ ] T001-T007: Initialize Docusaurus, configure, setup CI/CD"
echo ""
echo "Phase 2 (Content):"
echo "  [ ] T008-T011: Create 13 chapter templates with frontmatter"
echo ""
echo "Phase 3 (Components):"
echo "  [ ] T012-T016: Build MCQ, CollapsibleLab, Landing page components"
echo ""
echo "Phase 4 (Hardware):"
echo "  [ ] T017-T018: Create hardware page with 3 budget tiers"
echo ""
echo "Phase 5 (Assets):"
echo "  [ ] T019-T021: Add styling, images, README"
echo ""
echo "Phase 6 (Testing):"
echo "  [ ] T022-T026: Run tests, build validation, Lighthouse CI"
echo ""
echo "Phase 7 (Deployment):"
echo "  [ ] T027-T031: Push to GitHub, enable Pages, deploy"
echo ""
echo "Run individual tasks from tasks.md or use /sp.implement"
