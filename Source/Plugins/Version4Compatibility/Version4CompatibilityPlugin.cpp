/*
 * Your License or Copyright Information can go here
 */


#include "Version4CompatibilityPlugin.h"

#include "DREAM3DLib/Common/FilterManager.h"
#include "DREAM3DLib/Common/IFilterFactory.hpp"
#include "DREAM3DLib/Common/FilterFactory.hpp"


namespace Detail
{
  const QString Version4CompatibilityPluginFile("Version4CompatibilityPlugin");
  const QString Version4CompatibilityPluginDisplayName("Version4CompatibilityPlugin");
  const QString Version4CompatibilityPluginBaseName("Version4CompatibilityPlugin");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
Version4CompatibilityPlugin::Version4CompatibilityPlugin()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
Version4CompatibilityPlugin::~Version4CompatibilityPlugin()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString Version4CompatibilityPlugin::getPluginName()
{
  return (Detail::Version4CompatibilityPluginDisplayName);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void Version4CompatibilityPlugin::writeSettings(QSettings& prefs)
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void Version4CompatibilityPlugin::readSettings(QSettings& prefs)
{

}

#include "Version4CompatibilityFilters/RegisterKnownFilters.cpp"

#include "Version4Compatibility/FilterParameterWidgets/RegisterKnownFilterParameterWidgets.cpp"

