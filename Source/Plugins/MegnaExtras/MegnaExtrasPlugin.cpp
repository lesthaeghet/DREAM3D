/*
 * Your License or Copyright Information can go here
 */


#include "MegnaExtrasPlugin.h"

#include "DREAM3DLib/Common/FilterManager.h"
#include "DREAM3DLib/Common/IFilterFactory.hpp"
#include "DREAM3DLib/Common/FilterFactory.hpp"


#include "MegnaExtras/moc_MegnaExtrasPlugin.cpp"

Q_EXPORT_PLUGIN2(MegnaExtrasPlugin, MegnaExtrasPlugin)

namespace Detail
{
  const QString MegnaExtrasPluginFile("MegnaExtrasPlugin");
  const QString MegnaExtrasPluginDisplayName("MegnaExtrasPlugin");
  const QString MegnaExtrasPluginBaseName("MegnaExtrasPlugin");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
MegnaExtrasPlugin::MegnaExtrasPlugin()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
MegnaExtrasPlugin::~MegnaExtrasPlugin()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString MegnaExtrasPlugin::getPluginName()
{
  return (Detail::MegnaExtrasPluginDisplayName);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void MegnaExtrasPlugin::writeSettings(QSettings& prefs)
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void MegnaExtrasPlugin::readSettings(QSettings& prefs)
{

}

#include "MegnaExtrasFilters/RegisterKnownFilters.cpp"

#include "FilterParameterWidgets/RegisterKnownFilterParameterWidgets.cpp"

