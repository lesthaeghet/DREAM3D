/* ============================================================================
* Copyright (c) 2012 Michael A. Jackson (BlueQuartz Software)
* Copyright (c) 2012 Dr. Michael A. Groeber (US Air Force Research Laboratories)
* Copyright (c) 2012 Joseph B. Kleingers (Student Research Assistant)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice, this
* list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* Neither the name of Michael A. Groeber, Michael A. Jackson, Joseph B. Kleingers,
* the US Air Force, BlueQuartz Software nor the names of its contributors may be
* used to endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*  This code was written under United States Air Force Contract number
*                           FA8650-07-D-5800
*
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef _DREAM3DUserManualDialog_H_
#define _DREAM3DUserManualDialog_H_

#include "DREAM3DWidgetsLib/DREAM3DWidgetsLib.h"

#include <QtWidgets/QDialog>

#include "ui_DREAM3DUserManualDialog.h"

class DREAM3DWidgetsLib_EXPORT DREAM3DUserManualDialog : public QDialog, public Ui::DREAM3DUserManualDialog
{
  Q_OBJECT

public:

  /**
  * @brief Destructor
  * @return
  */
  virtual ~DREAM3DUserManualDialog();

  /**
  * @brief Static instance to retrieve the global instance of this class
  * @return
  */
  static DREAM3DUserManualDialog* Instance();

  /**
  * @brief Static function to launch the help dialog with the given URL
  * @return
  */
  static void LaunchHelpDialog(QUrl url);

  /**
  * @brief Static function to launch the help dialog with the given filter human label
  * @return
  */
  static void LaunchHelpDialog(QString humanLabel);

protected:
  DREAM3DUserManualDialog();

protected slots:
  void on_backBtn_pressed();
  void on_forwardBtn_pressed();
  void on_refreshBtn_pressed();
  void on_webView_loadFinished(bool ok);

private:
  static DREAM3DUserManualDialog* self;

  DREAM3DUserManualDialog(const DREAM3DUserManualDialog&); // Copy Constructor Not Implemented
  void operator=(const DREAM3DUserManualDialog&); // Operator '=' Not Implemented

};

#endif /* _DREAM3DUserManualDialog_H */

