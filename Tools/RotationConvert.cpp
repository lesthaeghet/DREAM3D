


#include <iostream>


#include <QtCore/QCoreApplication>
#include <QtCore/QString>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QMetaProperty>
#include <QtCore/QTextStream>
#include <QtCore/QDebug>


// DREAM3DLib includes
#include "DREAM3DLib/DREAM3DLib.h"
#include "DREAM3DLib/DREAM3DVersion.h"

#include "Tools/ToolConfiguration.h"


static bool inFunction = false;
static QString fType = QString("float");

static QStringList funcVars;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString quote(const QString& str)
{
  return QString("\"%1\"").arg(str);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void writeOutput(QStringList &outLines, QString filename)
{

  QFileInfo fi2(filename);
#if 1
  QFile hOut(filename);
#else
  QString tmpPath = "/tmp/" + fi2.fileName();
  QFile hOut(tmpPath);
#endif
  hOut.open(QFile::WriteOnly);
  QTextStream stream( &hOut );
  stream << outLines.join("\n");
  hOut.close();

  qDebug() << "Saved File " << fi2.absoluteFilePath();

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void generateHeader(QStringList& hOutLines)
{
  QString s;
  QTextStream out(&s);

  out << "#ifndef _RotationTransforms_H_\n";
  out << "#define _RotationTransforms_H_\n";
  out << "#include <string>\n";
  out << "#include \"DREAM3DLib/DREAM3DLib.h\"\n";

  out << "#include \"OrientationLib/OrientationLib.h\"\n";
  out << "class OrientationLib_EXPORT RotationTransforms\n{\n";
  out << "public:\n  RotationTransforms();\n  virtual ~RotationTransforms();\n";
  out << "void FatalError(const std::string &func, const std::string &msg);";
  hOutLines.append(s);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void generateFooter(QStringList& hOutLines)
{
  QString s;
  QTextStream out(&s);

  out << "};\n";
  out << "#endif /* _RotationTransforms_H_ */\n";
  hOutLines.append(s);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void generateImpl(QStringList& cOutLines)
{
  QString s;
  QTextStream out(&s);
  out << "#include \"RotationTransforms.h\"\n";
  out << "#include <iostream>\n";
  out << "#include \"DREAM3DLib/Math/DREAM3DMath.h\"\n";
  out << "\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "//\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "RotationTransforms::RotationTransforms()\n{\n}\n";
  out << "\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "//\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "RotationTransforms::~RotationTransforms()\n{\n}\n";
  out << "\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "//\n";
  out << "// -----------------------------------------------------------------------------\n";
  out << "void RotationTransforms::FatalError(const std::string &func, const std::string &msg)\n{\n";
  out << "  std::cout << func << \"::\" << msg << std::endl;\n";
  out << "}\n";

  cOutLines.append(s);
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool fixComment(const QString& line, QStringList& hOutLines, QStringList& cOutLines)
{
  bool res = false;
  QString tline = line.trimmed();
  if(line.startsWith('!'))
  {
    hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("module"))
  {
    //   hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("use"))
  {
    //    hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("interface"))
  {
    //   hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("end interface"))
  {
    //    hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("IMPLICIT"))
  {
    //    hOutLines.append("// " + line); res = true;
  }
  else if(tline.startsWith("public ::"))
  {
    //    hOutLines.append("// " + line); res = true;
  }
  //  else if(tline.startsWith("recursive function"))
  //  {
  //    hOutLines.append("// " + line); res = false;
  //  }
  else if(tline.isEmpty())
  {
    //hOutLines.append(line);
    res = true;
  }

  return res;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool fixFunctionDef(const QString& line, QStringList& hOutLines, QStringList& cOutLines)
{
  bool res = false;
  QString tline = line.trimmed();
  QString s;
  QTextStream out(&s);

  QString cpp;
  QTextStream cOut(&cpp);

  if(tline.startsWith("recursive function"))
  {
    res = true;
    inFunction = true;
    funcVars.clear();
    QStringList tokens = line.split(' ');
    out << "\n";
    out << "void ";
    QString f = tokens.at(2);
    QStringList fTokens = f.split('(');
    fType = "float";
    QString fName = fTokens.at(0);
    if(fName.endsWith("_d")) {
      fType = "double";
      fName = fName.mid(0, fName.length()-2);
    }
    QString inArg = fTokens.at(1);
    inArg = inArg.mid(0, inArg.length()-1);
    out << fName << "(" << fType << "* " << inArg << ", " << fType << "* res)";
    out << ";";

    if(s.contains("(float* av,omega, float* res)")) {
      s.replace("(float* av,omega, float* res)", "(float* av, float omega, float* res)");
    }
    if(s.contains("(double* av,omega, double* res)")) {
      s.replace("(double* av,omega, double* res)", "(double* av, double omega, double* res)");
    }
    if(s.contains("(float* orient,intype,rotcheck, float* res)")) {
      s.replace("(float* orient,intype,rotcheck, float* res)", "(float* orient, char intype[2], bool rotcheck, float* res)");
    }
    if(s.contains("(double* orient,intype,rotcheck, double* res)")) {
      s.replace("(double* orient,intype,rotcheck, double* res)", "(double* orient, char intype[2], bool rotcheck, double* res)");
    }
    if(s.contains("(float* vec,om,ap, float* res)")) {
      s.replace("(float* vec,om,ap, float* res)", "(float* vec, float* om, char ap, float* res)");
    }
    if(s.contains("(double* vec,om,ap, double* res)")) {
      s.replace("(double* vec,om,ap, double* res)", "(double* vec, double* om, char ap, double* res)");
    }

    if(s.contains("(float* vec,qu,ap, float* res)")) {
      s.replace("(float* vec,qu,ap, float* res)", "(float* vec, float* qu, char ap, float* res)");
    }
    if(s.contains("(double* vec,qu,ap, double* res)")) {
      s.replace("(double* vec,qu,ap, double* res)", "(double* vec, float* qu, char ap, double* res)");
    }
    if(s.contains("(float* tensor,om,ap, float* res)")) {
      s.replace("(float* tensor,om,ap, float* res)", "(float* tensor, float* om, char ap, float* res)");
    }
    if(s.contains("(double* tensor,om,ap, double* res)")) {
      s.replace("(double* tensor,om,ap, double* res)", "(double* tensor, double* om, char ap, double* res)");
    }

    hOutLines.append(s);

    s = "";
    out << "\n";
    out << "// -----------------------------------------------------------------------------\n";
    out << "//\n";
    out << "// -----------------------------------------------------------------------------\n";
    out << "void RotationTransforms::";
    out << fName << "(" << fType << "* " << inArg << ", " << fType << "* res)";
    if(s.contains("(float* av,omega, float* res)")) {
      s.replace("(float* av,omega, float* res)", "(float* av, float omega, float* res)");
    }
    if(s.contains("(double* av,omega, double* res)")) {
      s.replace("(double* av,omega, double* res)", "(double* av, double omega, double* res)");
    }
    if(s.contains("(float* orient,intype,rotcheck, float* res)")) {
      s.replace("(float* orient,intype,rotcheck, float* res)", "(float* orient, char intype[2], bool rotcheck, float* res)");
    }
    if(s.contains("(double* orient,intype,rotcheck, double* res)")) {
      s.replace("(double* orient,intype,rotcheck, double* res)", "(double* orient, char intype[2], bool rotcheck, double* res)");
    }
    if(s.contains("(float* vec,om,ap, float* res)")) {
      s.replace("(float* vec,om,ap, float* res)", "(float* vec, float* om, char ap, float* res)");
    }
    if(s.contains("(double* vec,om,ap, double* res)")) {
      s.replace("(double* vec,om,ap, double* res)", "(double* vec, double* om, char ap, double* res)");
    }

    if(s.contains("(float* vec,qu,ap, float* res)")) {
      s.replace("(float* vec,qu,ap, float* res)", "(float* vec, float* qu, char ap, float* res)");
    }
    if(s.contains("(double* vec,qu,ap, double* res)")) {
      s.replace("(double* vec,qu,ap, double* res)", "(double* vec, float* qu, char ap, double* res)");
    }
    if(s.contains("(float* tensor,om,ap, float* res)")) {
      s.replace("(float* tensor,om,ap, float* res)", "(float* tensor, float* om, char ap, float* res)");
    }
    if(s.contains("(double* tensor,om,ap, double* res)")) {
      s.replace("(double* tensor,om,ap, double* res)", "(double* tensor, double* om, char ap, double* res)");
    }
    cOutLines.append(s);
    cOutLines.append("{");
  }
  else if(tline.startsWith("end function"))
  {
    cOutLines.append("}");
    res = true;
    inFunction = false;
  }

  return res;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool functionConvert(const QString& cline, QStringList& hOutLines, QStringList& cOutLines)
{
  bool res = false;
  QString s;
  QTextStream out(&s);

  // Check for internal comment markers in the source line
  QString line = cline;
  if(line.contains(" !<"))
  {
    line = line.replace(" !<", " //");
    res = true;
  }
  if(line.contains(" ! "))
  {
    line = line.replace(" ! ", " //");
    res = true;
  }

  QString tline = line.trimmed();
  if(tline.contains("INTENT(IN)"))
  {
    QStringList tok = line.split("::");
    s = "";
    QStringList vars = tok.at(1).split(", ");
    foreach(QString var, vars)
    {
      if(var.startsWith(" res"))
      {

      }
      else
      {
        var = var.replace('(', '[');
        var = var.replace(')', ']');
        out << "  " << fType << " " << var << ";\n";
        int index = var.indexOf('[');
        int index2 = var.indexOf(']');
        if(index != -1 && index < var.length()) {
          bool ok = true;
          int k = var.mid(index + 1, index2 - index-1).toInt(&ok, 10);
          for(int m = 0; m < k; m++)
          {
            QString varName = var.mid(0, index).trimmed();
            varName = varName + "(" + QString::number(m+1) + ")";
            funcVars.push_back(varName);
          }
        }
      }
    }
    tline = "//"+line;
    res = true;
  }

  if( tline.startsWith("real(kind=sgl)") || tline.startsWith("real(kind=dbl)") )
  {
    QStringList tok = line.split("::");
    s = "";
    QStringList vars = tok.at(1).split(", ");
    foreach(QString var, vars)
    {
      if(var.startsWith(" res"))
      {

      }
      else
      {
        var = var.replace('(', '[');
        var = var.replace(')', ']');
        out << "  " << fType << " " << var << ";\n";
        int index = var.indexOf('[');
        int index2 = var.indexOf(']');
        if(index != -1 && index < var.length()) {
          bool ok = true;
          int k = var.mid(index + 1, index2 - index-1).toInt(&ok, 10);
          for(int m = 0; m < k; m++)
          {
            QString varName = var.mid(0, index).trimmed();
            varName = varName + "(" + QString::number(m+1) + ")";
            funcVars.push_back(varName);
          }
        }
      }
    }
    res = true;
    tline = s;
  }

  if(tline.startsWith("integer(kind=irg)"))
  {
    QStringList tok = line.split("::");
    if(tok.at(1).startsWith(" res") == false)
    {
      out << "  " << fType << tok.at(1) << ";";
      tline = s;
      s = "";
    }
    else
    {
      tline = "  //" + tline;
    }
    res = true;
  }

  if(tline.contains("sngl(cPi)"))
  {
    tline = tline.replace("sngl(cPi)", "M_PI");
    res = true;
  }
  if(tline.contains("cPi"))
  {
    tline = tline.replace("cPi", "M_PI");
    res = true;
  }
  if(tline.contains(".D0"))
  {
    tline = tline.replace(".D0", ".0");
    res = true;
  }
  if(tline.startsWith("if"))
  {
    tline = tline.replace(".eq.", " == ");
    tline = tline.replace(".lt.", " < ");
    tline = tline.replace(".gt.", " > ");
    tline = tline.replace(".or.", " || ");
    tline = tline.replace(".ne.", " != ");
    tline = tline.replace(".eqv..TRUE.", " == true ");
    tline = tline.replace(".eqv..FALSE.", " == false ");
    tline = tline.replace("then", "{");
    res = true;
  }
  if(tline.startsWith("end if") || tline.startsWith("endif"))
  {
    tline = "  }";
    res = true;
  }
  if(tline.startsWith("else"))
  {
    tline = "  } else {";
    res = true;
  }
  if(tline.startsWith("return"))
  {
    tline = " return;";
    res = true;
  }
  if(tline.startsWith("call"))
  {
    QStringList tok = tline.split("call");
    tline = tok.at(1).trimmed() + ";";
    res = true;
  }

  if(tline.startsWith("FatalError("))
  {
    tline = tline.replace("'", "\"");
    res = true;
  }

  if(tline.startsWith("res = 1"))
  {
    tline = "*"+tline + ";";
    res = true;
  }

  QStringList trigFuncs;
  trigFuncs << "tan" << "atan" << "atan2" << "sin" << "cos" << "sqrt" << "mod" << "acos" << "abs";

  foreach(QString trig, trigFuncs)
  {
    if(tline.contains("d"+trig+"(") )
    {
      tline = tline.replace("d"+trig+"(", trig+"(");
      res = true;
    }

  }
  // Look for variables that are really arrays and try to fix them up also
  foreach(QString var, funcVars)
  {
    if(tline.contains(var))
    {
      QString temp = var;
      temp = temp.replace('(', '[');
      temp = temp.replace(')', ']');

      int index = temp.indexOf('[');
      int index2 = temp.indexOf(']');
      if(index != -1 && index < var.length()) {
        bool ok = true;
        int k = var.mid(index + 1, index2 - index-1).toInt(&ok, 10);
        k = k - 1; // C++ is Zero Based Arrays
        temp = temp.left(index + 1) + QString::number(k) + "]";
        tline = tline.replace(var, temp);
        res = true;
      }
    }
  }

  if(line.startsWith("use") || line.startsWith("IMPLICIT"))
  {
    tline = "//*** " + line; res = true;
  }


  if(tline.contains("(float* av,omega, float* res)")) {
    tline.replace("(float* av,omega, float* res)", "(float* av, float omega, float* res)");
  }
  if(tline.contains("(double* av,omega, double* res)")) {
    tline.replace("(double* av,omega, double* res)", "(double* av, double omega, double* res)");
  }
  if(tline.contains("(float* orient,intype,rotcheck, float* res)")) {
    tline.replace("(float* orient,intype,rotcheck, float* res)", "(float* orient, char intype[2], bool rotcheck, float* res)");
  }
  if(tline.contains("(double* orient,intype,rotcheck, double* res)")) {
    tline.replace("(double* orient,intype,rotcheck, double* res)", "(double* orient, char intype[2], bool rotcheck, double* res)");
  }


  if(tline.contains("%"))
  {
    tline = tline.replace("%", ".");
    res = true;
  }

  if(res == true)
  {
    cOutLines.append(tline);
    res = true;
  }
  else
  {
    cOutLines.append(line + ";");
    res = true;
  }
  return res;
}



// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void convertFile1(const QString& fortFile, const QString hFile, const QString cppFile)
{
  QString contents;
  // Read the Source File
  QFileInfo fi(fortFile);
  QFile source(fortFile);
  source.open(QFile::ReadOnly);
  contents = source.readAll();
  source.close();

  QStringList hOutLines;

  generateHeader(hOutLines);
  QStringList cOutLines;
  generateImpl(cOutLines);

  QStringList list = contents.split(QRegExp("\\n"));
  QStringListIterator sourceLines(list);

  bool modded = false;
  while (sourceLines.hasNext())
  {
    QString line = sourceLines.next();

    if(modded == false)
      modded = fixComment(line, hOutLines, cOutLines);

    if(modded == false)
      modded = fixFunctionDef(line, hOutLines, cOutLines);

    if(modded == false && inFunction == true)
      modded = functionConvert(line, hOutLines, cOutLines);

    modded = false; // Reset this flag
  }

  generateFooter(hOutLines);
  writeOutput(hOutLines, hFile);
  writeOutput(cOutLines, cppFile);
}





// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  Q_ASSERT(false); // We don't want anyone to run this program.
  // Instantiate the QCoreApplication that we need to get the current path and load plugins.
  QCoreApplication app(argc, argv);
  QCoreApplication::setOrganizationName("BlueQuartz Software");
  QCoreApplication::setOrganizationDomain("bluequartz.net");
  QCoreApplication::setApplicationName("RotationConvert");

  std::cout << "RotationConvert Starting. Version " << DREAM3DLib::Version::PackageComplete().toStdString() << std::endl;

  QString fortFile("/Users/mjackson/Desktop/rotations/rotations.f90");
  QString hFile = D3DTools::GetDREAM3DSourceDir() + "/" + "OrientationLib/Math/RotationTransforms.h";
  QString cppFile = D3DTools::GetDREAM3DSourceDir() + "/" + "OrientationLib/Math/RotationTransforms.cpp";

  convertFile1(fortFile, hFile, cppFile);


  return 0;
}
