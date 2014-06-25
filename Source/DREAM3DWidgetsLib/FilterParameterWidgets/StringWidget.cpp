/* ============================================================================
 * Copyright (c) 2012 Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2012 Dr. Michael A. Groeber (US Air Force Research Laboratories)
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
 *                           FA8650-10-D-5210
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "StringWidget.h"

#include <QtCore/QMetaProperty>

#include <QtCore/QPropertyAnimation>
#include <QtCore/QSequentialAnimationGroup>
#include "DREAM3DWidgetsLib/DREAM3DWidgetsLibConstants.h"

#include "FilterParameterWidgetsDialogs.h"


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
StringWidget::StringWidget(FilterParameter* parameter, AbstractFilter* filter, QWidget* parent) :
  QWidget(parent),
  m_Filter(filter),
  m_FilterParameter(parameter)
{
  setupUi(this);
  setupGui();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
StringWidget::~StringWidget()
{}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::setupGui()
{


  blockSignals(true);
  if (m_FilterParameter != NULL)
  {
    QString units = m_FilterParameter->getUnits();
    if(units.isEmpty() == false)
    {
      label->setText(m_FilterParameter->getHumanLabel() + " (" + units + ")");
    }
    else
    {
      label->setText(m_FilterParameter->getHumanLabel() );
    }

    QString str = m_Filter->property(PROPERTY_NAME_AS_CHAR).toString();
    value->setText(str);
  }
  blockSignals(false);

  applyChangesBtn->setVisible(false);

  // Catch when the filter is about to execute the preflight
  connect(m_Filter, SIGNAL(preflightAboutToExecute()),
          this, SLOT(beforePreflight()));

  // Catch when the filter is finished running the preflight
  connect(m_Filter, SIGNAL(preflightExecuted()),
          this, SLOT(afterPreflight()));

  // Catch when the filter wants its values updated
  connect(m_Filter, SIGNAL(updateFilterParameters(AbstractFilter*)),
          this, SLOT(filterNeedsInputParameters(AbstractFilter*)));


  connect(value, SIGNAL(textChanged(const QString&)),
          this, SLOT(widgetChanged(const QString&)));

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::fadeInWidget(QWidget* widget)
{
  if (faderWidget)
  {
    faderWidget->close();
  }
  faderWidget = new FaderWidget(widget);
  faderWidget->start();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::on_value_returnPressed()
{
  on_applyChangesBtn_clicked();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::on_applyChangesBtn_clicked()
{
  value->setStyleSheet(QString(""));
  emit parametersChanged();

  if (faderWidget)
  {
    faderWidget->close();
  }
  faderWidget = new FaderWidget(applyChangesBtn);
  faderWidget->setFadeOut();
  connect(faderWidget, SIGNAL(animationComplete() ),
          this, SLOT(hideButton()));
  faderWidget->start();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::hideButton()
{
  value->setToolTip("");
  applyChangesBtn->setVisible(false);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::widgetChanged(const QString& text)
{
  value->setStyleSheet(QString::fromLatin1("color: rgb(255, 0, 0);"));
  value->setToolTip("Press the 'Return' key to apply your changes");
  if(applyChangesBtn->isVisible() == false)
  {
    applyChangesBtn->setVisible(true);
    fadeInWidget(applyChangesBtn);
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::filterNeedsInputParameters(AbstractFilter* filter)
{
  bool ok = filter->setProperty(PROPERTY_NAME_AS_CHAR, value->text());
  if(false == ok)
  {
    FilterParameterWidgetsDialogs::ShowCouldNotSetFilterParameter(m_Filter, m_FilterParameter);
  }

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::beforePreflight()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::afterPreflight()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::setLinkedConditionalState(int state)
{
  bool boolProp = (state == Qt::Checked);
  fadeWidget(this, boolProp);
}

#define FADE_TIME 160
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void StringWidget::fadeWidget(QWidget* widget, bool in)
{


#if 0
  if (in)
  {
    QPropertyAnimation* animation = new QPropertyAnimation(widget, "iconOpacity");
    animation->setDuration(FADE_TIME);
    animation->setEndValue(1.0);
    animation->start(QAbstractAnimation::DeleteWhenStopped);
  }
  else
  {
    QPropertyAnimation* animation = new QPropertyAnimation(this, "iconOpacity");
    animation->setDuration(FADE_TIME);
    animation->setEndValue(0.0);
    animation->start(QAbstractAnimation::DeleteWhenStopped);
  }
#else
  if (faderWidget)
  {
    faderWidget->close();
  }
  faderWidget = new FaderWidget(widget);
  if(in)
  {
    setVisible(true);
    faderWidget->setFadeIn();
    connect(faderWidget, SIGNAL(animationComplete() ),
            this, SLOT(show()));
  }
  else
  {
    faderWidget->setFadeOut();
    connect(faderWidget, SIGNAL(animationComplete() ),
            this, SLOT(hide()));
  }
  QColor color = DREAM3D::Defaults::BasicColor;
  if(m_FilterParameter->getAdvanced()) { color = DREAM3D::Defaults::AdvancedColor; }
  faderWidget->setStartColor(color);
  faderWidget->start();

#endif
}