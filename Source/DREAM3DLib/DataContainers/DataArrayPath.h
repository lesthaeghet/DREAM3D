/* ============================================================================
 * Copyright (c) 2014 Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2014 Dr. Michael A. Groeber (US Air Force Research Laboratories)
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
#ifndef _DataArrayPath_H_
#define _DataArrayPath_H_


#include <QtCore/QObject>
#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QStringList>

#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/Common/DREAM3DSetGetMacros.h"

/**
 * @brief The DataArrayPath class
 */
class DREAM3DLib_EXPORT DataArrayPath : public QObject
{
    Q_OBJECT

  public:
    DataArrayPath();

    /**
     * @brief DataArrayPath
     * @param dcName
     * @param amName
     * @param daName
     */
    DataArrayPath(const QString& dcName, const QString& amName, const QString& daName);

    /**
     * @brief DataArrayPath
     * @param path A path with the '|' delimeters
     */
    explicit DataArrayPath(const QString& path);

    /**
     * @brief DataArrayPath
     */
    DataArrayPath(const DataArrayPath& rhs);

    /**
     * @brief operator =
     */
    void operator=(const DataArrayPath&);

    virtual ~DataArrayPath();

    DREAM3D_INSTANCE_PROPERTY(QString, DataContainerName)
    DREAM3D_INSTANCE_PROPERTY(QString, AttributeMatrixName)
    DREAM3D_INSTANCE_PROPERTY(QString, DataArrayName)

    /**
     * @brief serialize Returns the path using the '|' charater by default. This can be over ridden by the programmer
     * @param delimiter
     * @return
     */
    QString serialize(QString delimiter = "|") const;

    QStringList getAsStringList();

    QVector<QString> getAsVector();

    /**
     * @brief isEmpty Returns if ALL of the string elements are empty. Note that a class could return FALSE for this
     * function and FALSE for isValid() also so this function is not a true indication of a valid path.
     * @return
     */
    bool isEmpty() const;

    /**
     * @brief isValid Returns if ALL of the string components have some value stored in them so 'valid' in this sense just
     * means that all three components have a non-empty string. This does NOT necessarily mean that those strings, once
     * serialized into a path actually refer to something in your DataContainer.
     * @return
     */
    bool isValid() const;

    /**
     * @brief split
     * @param delimiter
     * @return
     */
    QStringList split(QString NOT_USED = "|") const;

    /**
     * @brief Updates the DataArrayPath with the values in the arguments
     * @param dcName The DataContainer Name
     * @param amName The AttributeMatrix Name
     * @param daName The DataArray Name
     */
    void update(const QString& dcName, const QString& amName, const QString& daName);


  private:



};


Q_DECLARE_METATYPE(DataArrayPath)



#endif /* _DataArrayPath_H_ */