// ATomTrajForm.cpp : 實作檔
//

#include "stdafx.h"
#include "Robot_v3.h"
#include "ATomTrajForm.h"
#include "afxdialogex.h"
#include <string>
/////////////////////////
#include "ProjectPlay.h"
#include "AtomJointCtL.h"
#include "OpenGLControl.h"
// ATomTrajForm 對話方塊
using namespace std;
int setpointcount;
IMPLEMENT_DYNAMIC(ATomTrajForm, CDialogEx)

ATomTrajForm::ATomTrajForm(CWnd* pParent /*=NULL*/)
	: CDialogEx(ATomTrajForm::IDD, pParent)
{
}

ATomTrajForm::~ATomTrajForm()
{
}

void ATomTrajForm::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST2, CListBox_AtomSetPoint);
	DDX_Control(pDX, IDC_TrajSelect, IDC_AtomTrajSelec);
	DDX_Control(pDX, IDC_EditTime, CEdit_AtomSetTime);
	DDX_Control(pDX, IDC_ClearListBox, mClearListBoxBtn);
}


BEGIN_MESSAGE_MAP(ATomTrajForm, CDialogEx)
	ON_BN_CLICKED(IDC_AtomTrajGO, &ATomTrajForm::OnBnClickedAtomtrajgo)
	ON_BN_CLICKED(IDC_AtomSet, &ATomTrajForm::OnBnClickedAtomset)
	ON_LBN_SELCHANGE(IDC_LIST2, &ATomTrajForm::OnLbnSelchangeList2)
	ON_CBN_SELCHANGE(IDC_TrajSelect, &ATomTrajForm::OnCbnSelchangeTrajselect)
	ON_EN_CHANGE(IDC_EditTime, &ATomTrajForm::OnEnChangeEdittime)
	ON_CBN_SELENDOK(IDC_TrajSelect, &ATomTrajForm::OnCbnSelendokTrajselect)
	ON_BN_CLICKED(IDC_ClearListBox, &ATomTrajForm::OnBnClickedClearlistbox)
END_MESSAGE_MAP()


// ATomTrajForm 訊息處理常式
BOOL ATomTrajForm::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	AtomJointCtL mAtomJointCtL;
	setpointcount = 1;
	//Initial ListBox
	CString setPointstring;
	setPointstring.Format("%d.P(x,y,z,rx,ry,rz)=(%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)",
		setpointcount,
		mAtomJointCtL.Atom_CarAll[0],
		mAtomJointCtL.Atom_CarAll[1],
		mAtomJointCtL.Atom_CarAll[2],
		mAtomJointCtL.Atom_CarAll[3],
		mAtomJointCtL.Atom_CarAll[4],
		mAtomJointCtL.Atom_CarAll[5]);

	CListBox_AtomSetPoint.AddString(setPointstring);
	// Initial time listbox
	SetDlgItemText(IDC_EditTime,"3");
	// Initial 下拉式選單
	IDC_AtomTrajSelec.ResetContent();
	IDC_AtomTrajSelec.InsertString(0,"1. 212");
	IDC_AtomTrajSelec.InsertString(1,"2. 434");
	IDC_AtomTrajSelec.InsertString(2,"3. 535");
	IDC_AtomTrajSelec.InsertString(3,"4. Cubic");
	IDC_AtomTrajSelec.InsertString(4,"5. Line");
	IDC_AtomTrajSelec.InsertString(5,"6. Simple");
	IDC_AtomTrajSelec.SetCurSel(5);
	return true;
}
void ATomTrajForm::OnBnClickedAtomtrajgo()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	ProjectPlay mProject;
	mProject.Atom_TrajectoryGO();
}
void ATomTrajForm::OnBnClickedAtomset()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	ProjectPlay mProject;
	CEdit *timeEdit;
	timeEdit = (CEdit*)GetDlgItem(IDC_EditTime);  // get time CEDIT value
	CString str;
	float times;
	timeEdit->GetWindowTextA(str);   // CEdit to string
	times = atof(str);  //  string to float
	setpointcount++;
	//////////////////////////////////////////////////////
	CString setPointstring,TrajType;
	int index_ = IDC_AtomTrajSelec.GetCurSel();
	IDC_AtomTrajSelec.GetLBText(index_,TrajType);
	setPointstring.Format("%d.Type:%s,Ts:%.1f P=(%.3f,%.3f,%.3f,%.1f,%.1f,%.1f)",
		setpointcount,
		TrajType,
		times,
		AtomJointCtL::Atom_CarAll[0],
		AtomJointCtL::Atom_CarAll[1],
		AtomJointCtL::Atom_CarAll[2],
		AtomJointCtL::Atom_CarAll[3],
		AtomJointCtL::Atom_CarAll[4],
		AtomJointCtL::Atom_CarAll[5]);

	CListBox_AtomSetPoint.AddString(setPointstring);
	mProject.Atom_Setpoint(AtomJointCtL::Atom_thetaALL,times,index_);
	////////////////////////////////////////////////////
	//COpenGLControl mOpenGLControl;
	//mOpenGLControl.Draw_sphere(0.05,mAtomJointCtL.Atom_CarAll[0],mAtomJointCtL.Atom_CarAll[1],mAtomJointCtL.Atom_CarAll[2]);

}
void ATomTrajForm::OnLbnSelchangeList2()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	UpdateWindow();
}
void ATomTrajForm::OnCbnSelchangeTrajselect()
{
	// TODO: 在此加入控制項告知處理常式程式碼
}
void ATomTrajForm::OnEnChangeEdittime()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialogEx::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
}

void ATomTrajForm::OnCbnSelchangeCombo3()
{
	// TODO: 在此加入控制項告知處理常式程式碼
}


void ATomTrajForm::OnCbnSelendokTrajselect()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	CString str;
	int index = IDC_AtomTrajSelec.GetCurSel();
}


void ATomTrajForm::OnBnClickedClearlistbox()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	ProjectPlay mProjectPlay;
	mProjectPlay.Atom_TrajClear();
	CListBox_AtomSetPoint.ResetContent();
	ASSERT(CListBox_AtomSetPoint.GetCount()==0);
	OnInitDialog();
}
