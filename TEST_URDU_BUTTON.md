# Urdu Translation Button - Integration Test Guide

## Pre-Test Checklist

- [ ] Backend dependencies installed: `pip install -r requirements.txt`
- [ ] `.env.local` file created with `GROQ_API_KEY=gsk_...`
- [ ] Groq API key is valid (test at https://console.groq.com)
- [ ] Frontend dependencies installed: `npm install`
- [ ] Node.js version >=20.0

---

## Step 1: Start Backend Server

```bash
cd backend
source venv/bin/activate  # Windows: venv\Scripts\activate
uvicorn app.main:app --reload --port 8003
```

### Expected Output
```
INFO:     Uvicorn running on http://0.0.0.0:8003
INFO:     Application startup complete
```

### Verify Backend Health
```bash
curl http://localhost:8003/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "model": "groq/llama-3.1-70b-instant",
  "agent": "OpenAI Agents SDK + LiteLLM"
}
```

---

## Step 2: Test Backend API Directly

### Test 1: Simple Translation Request

```bash
curl -X POST http://localhost:8003/api/v1/translate \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "ch01",
    "content": "Physical AI is the study of artificial intelligence in robots and physical systems. It combines embodied cognition, sensor-based perception, and motor control.",
    "skipCache": false
  }'
```

**Expected Response**:
- Status: `200 OK`
- Contains `urduContent` field with Urdu text
- Contains `duration` field (should be 1000-2000 ms for first request)
- Contains `cached: false` (first request)

### Test 2: Cache Hit Test

Run the same request again:

```bash
curl -X POST http://localhost:8003/api/v1/translate \
  -H "Content-Type: application/json" \
  -d '{
    "chapterId": "ch01",
    "content": "Physical AI is the study of artificial intelligence in robots and physical systems. It combines embodied cognition, sensor-based perception, and motor control.",
    "skipCache": false
  }'
```

**Expected Response**:
- Status: `200 OK`
- Contains same `urduContent`
- Contains `duration` field (should be 0-10 ms - very fast!)
- Contains `cached: true` (from cache)

### Test 3: Cache Status Check

```bash
curl http://localhost:8003/api/v1/translate/ch01/status
```

**Expected Response**:
```json
{
  "chapterId": "ch01",
  "isCached": true,
  "cacheExpiry": "2025-01-09T...",
  "age": "1m"
}
```

### Test 4: View Cache Statistics

```bash
curl http://localhost:8003/api/v1/cache/stats | jq
```

**Expected Output**:
```json
{
  "timestamp": "2025-12-10T...",
  "cacheStats": {
    "totalEntries": 1,
    "totalSizeKB": 45,
    "entries": [
      {
        "chapterId": "ch01",
        "size": 45,
        "age": "2m",
        "expiryTime": "2025-01-09T..."
      }
    ]
  }
}
```

---

## Step 3: Start Frontend Server

In a NEW terminal window:

```bash
cd docusaurus
npm start
```

### Expected Output
```
[info] ✨ Ready in 5.23 s
[info] ℹ️  Docusaurus website is running at: http://localhost:3000/
```

---

## Step 4: Test Frontend UI

### Test 1: Button Visibility

1. Open http://localhost:3000 in browser
2. Navigate to **Chapter 1: Introduction to Physical AI**
3. **Verify**: Blue button visible near top of page with text "اردو میں دیکھیں" (View in Urdu)

### Test 2: Translation on Click

1. Click the blue button
2. **Verify**: Button shows "Translating..." with spinner
3. Wait 1-2 seconds
4. **Verify**: Content changes to Urdu (right-to-left layout)
5. **Verify**: Button changes to green with text "Show in English"

### Test 3: Toggle Back to English

1. Click the green button
2. **Verify**: Content switches back to English
3. **Verify**: Button changes back to blue "اردو میں دیکھیں"

### Test 4: Fast Cached Load

1. Click blue button again
2. **Verify**: Content switches to Urdu almost instantly (<500ms)
3. **Verify**: Check browser console (F12 → Console) for log message:
   ```
   Translation for ch01: 5ms (cached: true)
   ```

### Test 5: Code Blocks Preserved

1. Translate any chapter with code examples (e.g., Chapter 2 - ROS 2)
2. **Verify**: Python code blocks, ROS commands, etc. are NOT translated
3. **Verify**: Code blocks are still in English within Urdu content

### Test 6: Technical Terms in English

1. Translate Chapter 1 (Physical AI)
2. **Verify**: Technical terms remain in English:
   - "ROS 2" (not translated)
   - "Isaac Sim" (not translated)
   - "URDF" (not translated)
   - "Gazebo" (not translated)
   - Regular words in English are translated to Urdu

### Test 7: Mobile Responsiveness

1. Open Chapter 1 in browser
2. Press F12 (Developer Tools)
3. Click device toggle (mobile view)
4. Change to iPhone/mobile resolution
5. **Verify**: Button is visible and clickable
6. **Verify**: Translation works on mobile

### Test 8: Multiple Chapters

Test chapters 1, 5, and 13:

1. Navigate to Chapter 1 → Translate → Verify Urdu
2. Go to Chapter 5 → Translate → Verify Urdu (different content)
3. Go to Chapter 13 → Translate → Verify Urdu
4. Back to Chapter 1 → Content should still be in Urdu (language state per chapter)

---

## Step 5: Test Error Scenarios

### Test 1: Offline Mode

1. Open Developer Tools (F12)
2. Go to Network tab
3. Check "Offline" checkbox
4. Try to translate a new chapter
5. **Verify**: Error message appears: "Translation not available"
6. **Verify**: English content displayed (not broken)
7. Uncheck "Offline"
8. Try again
9. **Verify**: Translation works again

### Test 2: Invalid API Key

1. Stop backend server (Ctrl+C)
2. Edit `.env.local`: Change `GROQ_API_KEY=invalid_key`
3. Restart backend
4. Try to translate new chapter (ch02)
5. **Verify**: Error message appears: "Service unavailable" or API error
6. **Verify**: User-friendly error message (not raw API error)

### Test 3: Rate Limit (Groq Free Tier)

Groq free tier: 30 requests/min, 6000 tokens/min

1. If you hit rate limit, you'll see error: "Too many requests"
2. **Verify**: Retry message appears
3. Wait 60 seconds
4. Try again
5. **Verify**: Works after waiting

---

## Step 6: Performance Testing

### Measure First Translation Time

1. Open Developer Tools (F12)
2. Go to Network tab
3. Click translate button on Chapter 1
4. **Note**: Time for POST to `/api/v1/translate`
5. **Verify**: Should be 1000-2000ms (Groq API latency)

### Measure Cached Load Time

1. Translate Chapter 1 (done in Step 4)
2. Translate Chapter 1 again
3. **Verify**: Cache hit (<10ms for API, total <500ms with rendering)

### Check Console Logs

1. Press F12 → Console tab
2. Translate a chapter
3. **Verify**: See log message:
   ```
   Translation for ch01: 1234ms (cached: false)
   ```

---

## Step 7: Cross-Browser Testing

### Chrome
1. Open http://localhost:3000
2. Translate Chapter 1
3. **Verify**: Works

### Firefox
1. Open http://localhost:3000
2. Translate Chapter 2
3. **Verify**: Works

### Safari (macOS)
1. Open http://localhost:3000
2. Translate Chapter 3
3. **Verify**: Works

---

## Final Verification Checklist

- [ ] Backend health check returns 200
- [ ] Backend API returns valid Urdu translation
- [ ] Cache hit/miss works correctly
- [ ] Frontend button visible on all 13 chapters
- [ ] Translation loads in <2 seconds (first request)
- [ ] Cached translation loads in <500ms
- [ ] Urdu text displays correctly (RTL layout)
- [ ] Toggle between English/Urdu works
- [ ] Code blocks preserved (not translated)
- [ ] Technical terms in English (ROS 2, Isaac Sim, etc.)
- [ ] Error messages user-friendly
- [ ] Offline mode handled gracefully
- [ ] Mobile responsive
- [ ] Works on Chrome, Firefox, Safari

---

## Common Issues & Solutions

### Issue: Button not visible
**Solution**:
- Check console (F12) for errors
- Verify import statement in chapter file
- Clear browser cache: Ctrl+Shift+Delete
- Rebuild: `npm run build`

### Issue: Translation fails silently
**Solution**:
- Check backend is running: `curl http://localhost:8003/health`
- Check `.env.local` has valid `GROQ_API_KEY`
- Check backend logs for errors
- Verify Groq account is active

### Issue: Very slow translation (>5 seconds)
**Solution**:
- Check internet connection
- Check Groq API status: https://console.groq.com/status
- Check backend logs for network issues
- Try smaller chapter content

### Issue: "CORS error" in browser console
**Solution**:
- Verify backend CORS is enabled (it is by default)
- Restart backend server
- Check `REACT_APP_API_URL` env var (should be http://localhost:8003)

---

## Success Criteria

✅ **Feature is working correctly if all tests pass**:

1. ✅ Backend translation API responds in <2s
2. ✅ Cached responses <500ms
3. ✅ Frontend button visible on all 13 chapters
4. ✅ Translation displays correctly (Urdu, RTL)
5. ✅ Toggle between languages works
6. ✅ Code blocks NOT translated
7. ✅ Technical terms preserved in English
8. ✅ Error handling graceful
9. ✅ Works across browsers and mobile
10. ✅ Performance meets targets

---

## Next Steps

After successful testing:

1. **Deploy Backend**:
   - Production: Use Gunicorn + Nginx
   - Or: Deploy to Railway/Vercel
   - Set real `GROQ_API_KEY` in production

2. **Deploy Frontend**:
   - Build: `npm run build`
   - Deploy to GitHub Pages: `npm run deploy`
   - Or: Deploy to Vercel/Netlify

3. **Monitor**:
   - Watch backend logs
   - Monitor API response times
   - Track cache hit rates

4. **Gather Feedback**:
   - Get Urdu-speaking users to review translation quality
   - Iterate on prompt engineering if needed
   - Track user satisfaction (optional: add rating button)

---

**Test Date**: _______________
**Tester**: _______________
**Result**: ✅ PASS / ❌ FAIL

**Notes**: _______________
