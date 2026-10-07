#include "source_select.h"
#include "sensor_types.h"

const char *sensor_source_str(uint8_t source)
{
    switch (source)
    {
        case SRC_VN200:  return "VN200";
        case SRC_BNO085: return "BNO085";
        case SRC_BAR30:  return "Bar30";
        case SRC_ADC:    return "ADC";
        default:         return "none";
    }
}

void source_select_init(SourceSelect *s, uint8_t preferred, uint8_t backup)
{
    s->preferred             = preferred;
    s->backup                = backup;
    s->active                = SRC_NONE;
    s->preferred_was_fresh   = false;
    s->preferred_fresh_since = 0U;
}

/* Unsigned subtraction is correct across the 32-bit tick wrap. */
static bool fresh(uint32_t now, bool seen, uint32_t last)
{
    return seen && ((now - last) <= SOURCE_STALE_MS);
}

uint8_t source_select_update(SourceSelect *s, uint32_t now_ms,
                             bool preferred_seen, uint32_t preferred_last_ms,
                             bool backup_seen,    uint32_t backup_last_ms)
{
    bool pref_ok   = fresh(now_ms, preferred_seen, preferred_last_ms);
    bool backup_ok = fresh(now_ms, backup_seen, backup_last_ms);

    /* Start of an unbroken run of fresh readings from the preferred one. */
    if (pref_ok && !s->preferred_was_fresh)
    {
        s->preferred_fresh_since = now_ms;
    }
    s->preferred_was_fresh = pref_ok;

    bool pref_settled = pref_ok &&
                        ((now_ms - s->preferred_fresh_since) >= SOURCE_RETURN_MS);

    if (s->active == s->preferred)
    {
        if (!pref_ok)
        {
            s->active = backup_ok ? s->backup : (uint8_t)SRC_NONE;
        }
    }
    else if (s->active == s->backup)
    {
        if (pref_settled || (pref_ok && !backup_ok))
        {
            s->active = s->preferred;
        }
        else if (!backup_ok)
        {
            s->active = SRC_NONE;
        }
    }
    else
    {
        /* Nothing active yet (boot), or both were lost: take whatever is
         * fresh now, preferred first. No waiting - something is better
         * than nothing. */
        if (pref_ok)
        {
            s->active = s->preferred;
        }
        else if (backup_ok)
        {
            s->active = s->backup;
        }
    }

    return s->active;
}
