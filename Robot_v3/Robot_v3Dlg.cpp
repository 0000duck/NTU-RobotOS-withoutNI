
// Robot_v3Dlg.cpp : implementation file
//
#include "stdafx.h"
#include "Robot_v3.h"
#include "Robot_v3Dlg.h"
#include "global_value.h"
#include "ProjectPlay.h"
//3RD party Control panel
#include "AtomJointCtL.h"
#include "AtomControl.h"
#include "ATomTrajForm.h"
#include "GripperControl.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
bool GLDrawAtomStickFlag = false;  //  draw the Robot Arm stick
bool ForceSensorClose = true;
HANDLE gSendingThread; // arm control thread handle
DWORD gTIDSending;//  arm control thread ID
bool gSendingLife = true;//  arm control 是否工作旗標
bool gSendingThreadOpened = false; // 指示手臂部控制thread是否打開
void commandThreadControl( bool shallWe);
float gTimer = 5.0; // 5ms
// Open a console
void CRobot_v3Dlg::InitConsole()
{
	int nRet = 0;
	FILE* fp;
	AllocConsole();
	nRet = _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	fp = _fdopen(nRet, "w");
	*stdout = *fp;
	setvbuf(stdout, NULL, _IONBF, 0);
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)

END_MESSAGE_MAP()


// CRobot_v3Dlg dialog


CRobot_v3Dlg::CRobot_v3Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRobot_v3Dlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobot_v3Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_OpenGLScene, OpenGLSceneMainF);
	DDX_Control(pDX, IDC_BUTTON1, ProjectTest);
	DDX_Control(pDX, IDC_BUTTON2, Atom_Joint_Btn);
	DDX_Control(pDX, IDC_CHECK_Atom, check_atom);
	DDX_Control(pDX, IDC_NTU8DOFBtn, NTU8DOFbtn);
	DDX_Control(pDX, IDC_CHECK_NTU8DOF, Check_NTU8DOF);
	DDX_Control(pDX, IDC_AtomCtL, Atom_Cont_Btn);
	DDX_Control(pDX, IDC_AtomTraj, Atom_Traj_Btn);
	DDX_Control(pDX, IDC_Gripper, Gripper_Btn);
}

BEGIN_MESSAGE_MAP(CRobot_v3Dlg, CDialogEx)
//	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_QUERYDRAGICON()
	ON_WM_MOUSEWHEEL()
	ON_BN_CLICKED(IDC_BUTTON1, &CRobot_v3Dlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CRobot_v3Dlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_CHECK_Atom, &CRobot_v3Dlg::OnBnClickedCheckAtom)
	ON_BN_CLICKED(IDC_NTU8DOFBtn, &CRobot_v3Dlg::OnBnClickedNtu8dofbtn)
	ON_BN_CLICKED(IDC_CHECK_NTU8DOF, &CRobot_v3Dlg::OnBnClickedCheckNtu8dof)
	ON_BN_CLICKED(IDC_AtomCtL, &CRobot_v3Dlg::OnBnClickedAtomctl)
	ON_BN_CLICKED(IDC_AtomTraj, &CRobot_v3Dlg::OnBnClickedAtomtraj)
	ON_WM_SYSCOMMAND()
	ON_BN_CLICKED(IDC_Gripper, &CRobot_v3Dlg::OnBnClickedGripper)
END_MESSAGE_MAP()


// CRobot_v3Dlg message handlers

BOOL CRobot_v3Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.	

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	InitConsole();  //  open the console
	//*********Opengl Control*************
	CRect rect;

	OpenGLSceneMainF.GetWindowRect(rect);
	ScreenToClient(rect);

	m_OpenGLControl.openGLCreate(rect,this);
	m_OpenGLControl.Dwidth = rect.right-rect.left;
	m_OpenGLControl.Dhigh = rect.bottom-rect.top;

	// Setup the OpenGL Window's timer to render
	m_OpenGLControl.m_unpTimer = m_OpenGLControl.SetTimer(1,1,0);

	//*********Opengl Control end*************

	return TRUE;  // return TRUE  unless you set the focus to a control
}
void CRobot_v3Dlg::OnOK(void)
{
	CWnd* pWnd = GetFocus();
	if(GetDlgItem(IDOK) == pWnd)
	{
		CDialog::OnOK();
		return;
	}
	// Enter key was hit -> do whatever you want
}
//void CRobot_v3Dlg::OnSysCommand(UINT nID, LPARAM lParam)
//{
//	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
//	{
//		CAboutDlg dlgAbout;
//		dlgAbout.DoModal();
//	}
//	else
//	{
//		CDialogEx::OnSysCommand(nID, lParam);
//	}
//}
void CRobot_v3Dlg::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);
	switch (nType)
	{
	case SIZE_RESTORED:
		if (this->m_OpenGLControl.m_bIsMaximized)
		{
			this->m_OpenGLControl.OnSize(nType,cx,cy);
			this->m_OpenGLControl.m_bIsMaximized = false;
		}
		break;
	case SIZE_MAXIMIZED:
		this->m_OpenGLControl.OnSize(nType,cx,cy);
		this->m_OpenGLControl.m_bIsMaximized = true;
		break;

	}
}
BOOL CRobot_v3Dlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	this->m_OpenGLControl.OnMouseWheel(nFlags, zDelta,pt);
	return CDialog::OnMouseWheel(nFlags,zDelta,pt);
}
// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRobot_v3Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRobot_v3Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CRobot_v3Dlg::OnBnClickedButton1()
{
	ProjectPlay mProjectplay;   //  sub-Project C++ can write anything... 
	//mProjectplay.AtomDynControl();
	//m_OpenGLControl.a = true;
}


void CRobot_v3Dlg::OnBnClickedButton2()
{
//	AtomJointCtL m_AtomJointCtL;
//	m_AtomJointCtL.DoModal();
	AtomJointCtL *pDlg = new AtomJointCtL();
	pDlg->Create(AtomJointCtl,this);
	pDlg->ShowWindow(SW_NORMAL);
}


void CRobot_v3Dlg::OnBnClickedCheckAtom()
{
	//UINT nCheck = check_atom.GetState();
	ProjectPlay mProjectplay;   //  sub-Project C++ can write anything... 
	int nCheck = check_atom.GetCheck();
	if (nCheck)
	{
		//Atom Initial
		mProjectplay.project_InitAtom();  //  啟動project.cpp內的資訊
		Atom_Joint_Btn.EnableWindow(true);
		Atom_Cont_Btn.EnableWindow(true);
		Atom_Traj_Btn.EnableWindow(true);
		Gripper_Btn.EnableWindow(true);
	}
	else
	{
		mProjectplay.project_ATomThreadClose();
		Atom_Joint_Btn.EnableWindow(false);
		Atom_Cont_Btn.EnableWindow(false);
		Atom_Traj_Btn.EnableWindow(false);
		Gripper_Btn.EnableWindow(false);
	}
	UpdateData(true);
}


void CRobot_v3Dlg::OnBnClickedNtu8dofbtn()
{
	// TODO: 在此加入控制項告知處理常式程式碼
}

void CRobot_v3Dlg::OnBnClickedCheckNtu8dof()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	UpdateData(true);
	ProjectPlay mProjectplay;   //  sub-Project C++ can write anything... 
	if (Check_NTU8DOF)
	{
		//Atom Initial
		mProjectplay.ProjectPlay_InitNTU8DOF();
		NTU8DOFbtn.EnableWindow(true);
	}
	else
	{
		NTU8DOFbtn.EnableWindow(FALSE);
	}

}


void CRobot_v3Dlg::OnBnClickedAtomctl()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	//AtomControl mAtomControl;
	//mAtomControl.DoModal();
	AtomControl *pDlg = new AtomControl();
	pDlg->Create(IDD_AtomControl,this);
	pDlg->ShowWindow(SW_NORMAL);

	

}


void CRobot_v3Dlg::OnBnClickedAtomtraj()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	ATomTrajForm *pDlg = new ATomTrajForm();
	pDlg->Create(IDD_AtomTrajForm,this);
	pDlg->ShowWindow(SW_NORMAL);
}


void CRobot_v3Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值

	CDialogEx::OnSysCommand(nID, lParam);
}


void CRobot_v3Dlg::OnBnClickedGripper()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	GripperControl *pDlg = new GripperControl();
	pDlg->Create(IDD_Gripper,this);
	pDlg->ShowWindow(SW_NORMAL);
}