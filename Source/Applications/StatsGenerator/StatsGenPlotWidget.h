/* ============================================================================
 * Copyright (c) 2010, Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2010, Dr. Michael A. Groeber (US Air Force Research Laboratories
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
 * Neither the name of Michael A. Groeber, Michael A. Jackson, the US Air Force,
 * BlueQuartz Software nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written
 * permission.
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

#ifndef _STATSGENPLOTWIDGET_H_
#define _STATSGENPLOTWIDGET_H_

#include <QtWidgets/QWidget>

#include "ui_StatsGenPlotWidget.h"

#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/Common/DREAM3DSetGetMacros.h"
#include "DREAM3DLib/Common/Constants.h"
#include "DREAM3DLib/StatsData/StatsData.h"
#include "DREAM3DLib/DataContainers/VolumeDataContainer.h"


class SGAbstractTableModel;
class QwtPlotZoomer;
class QwtPlotPicker;
class QwtPlotPanner;
class QwtPlotGrid;
class QwtPlotCurve;

namespace UIA
{
  const static int Alpha = 255;
}

class StatsGenPlotWidget : public QWidget, private Ui::StatsGenPlotWidget
{

    Q_OBJECT

  public:


    StatsGenPlotWidget(QWidget* parent = 0);
    virtual ~StatsGenPlotWidget();

    void setPlotTitle(QString title);

    VectorOfFloatArray getStatisticsData();

    int extractStatsData(int index,
                         QVector<float>&  binNumbers,
                         VectorOfFloatArray arrays);

    void setStatisticsType(unsigned int statsType);
    void setDistributionType(unsigned int distributionType, bool updatePlots = true);
    int getDistributionType() { return m_DistributionType; }
    void blockDistributionTypeChanges(bool block);

    void setXAxisName(QString name);
    void setYAxisName(QString name);

    void setRowOperationEnabled(bool b);

    void setupGui();

    void createBetaCurve(int tableRow, float& xMax, float& yMax);
    void createLogNormalCurve(int tableRow, float& xMax, float& yMax);
    void createPowerCurve(int tableRow, float& xMax, float& yMax);
    SGAbstractTableModel* tableModel();

//    void setBins(QVector<float> &binValues);
    bool userUpdatedData();

    DREAM3D_INSTANCE_PROPERTY(float, Mu)
    DREAM3D_INSTANCE_PROPERTY(float, Sigma)
    DREAM3D_INSTANCE_PROPERTY(float, MinCutOff)
    DREAM3D_INSTANCE_PROPERTY(float, MaxCutOff)
    DREAM3D_INSTANCE_PROPERTY(float, BinStep)

    DREAM3D_INSTANCE_PROPERTY(int, PhaseIndex)
    DREAM3D_INSTANCE_PROPERTY(unsigned int, CrystalStructure)


    void setSizeDistributionValues(float mu, float sigma,
                                   float minCutOff, float maxCutOff,
                                   float binStepSize);


  signals:
    void userEditedData();

  protected slots:
    void updatePlotCurves();
    void userCommittedData(QWidget* w);
    // Auto Hook up Slots
    void on_addRowBtn_clicked();
    void on_deleteRowBtn_clicked();
    void on_distributionTypeCombo_currentIndexChanged(int index);

  protected:
    void resetTableModel();


  private:
    unsigned int m_DistributionType;
    SGAbstractTableModel* m_TableModel;
    QwtPlotGrid* m_grid;
    unsigned int m_StatsType;
    bool m_UserUpdatedData;

    QVector<QwtPlotCurve* > m_PlotCurves;

    StatsGenPlotWidget(const StatsGenPlotWidget&); // Copy Constructor Not Implemented
    void operator=(const StatsGenPlotWidget&); // Operator '=' Not Implemented
};

#endif /* _STATSGENPLOTWIDGET_H_ */

