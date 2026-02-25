/*
 *  prefs_items.h - Common preferences items
 *
 *  Basilisk II (C) 1997-2008 Christian Bauer
 */

#ifndef PREFS_ITEMS_H
#define PREFS_ITEMS_H

#include "prefs.h"

// Common preferences items (defined in prefs_items.cpp)
extern prefs_desc common_prefs_items[];

// Set default values for preferences items
extern void AddPrefsDefaults(void);

#endif /* PREFS_ITEMS_H */
