namespace WindowsApplication1
{
    partial class frmMain
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.menuStrip1 = new System.Windows.Forms.MenuStrip();
            this.fileToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.exitToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.viewToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.trackedFeaturesToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.searchEllipsesToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.overheadMToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.particleProbabilitiesToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.augmentedRealityToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.videoToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.startSentienceToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.runSimulationToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.toolsToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.calibrationToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.recordImagesToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.picLeftImage = new System.Windows.Forms.PictureBox();
            this.picOutput1 = new System.Windows.Forms.PictureBox();
            this.grpSettings = new System.Windows.Forms.GroupBox();
            this.txtCaptureTime = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.lblFrameRateWarning = new System.Windows.Forms.Label();
            this.lblTracking = new System.Windows.Forms.Label();
            this.txtProcessingTime = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.cmdBeginTracking = new System.Windows.Forms.Button();
            this.txtSpeed = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.txtfps = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.timUpdate = new System.Windows.Forms.Timer(this.components);
            this.panelCalibration = new System.Windows.Forms.Panel();
            this.cmdCalibrationDone = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.txtTargetHeight = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.txtTargetWidth = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.txtDistToTarget = new System.Windows.Forms.TextBox();
            this.cmdCentreDistortionYdown = new System.Windows.Forms.Button();
            this.cmdCentreDistortionYup = new System.Windows.Forms.Button();
            this.cmdCentreDistortionXDown = new System.Windows.Forms.Button();
            this.cmdCentreDistortionXUp = new System.Windows.Forms.Button();
            this.cmdDistortionDown = new System.Windows.Forms.Button();
            this.cmdDistortionUp = new System.Windows.Forms.Button();
            this.label6 = new System.Windows.Forms.Label();
            this.txtCentreOfDistortionY = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.txtCentreOfDistortionX = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.txtDistortion = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.txtFOV = new System.Windows.Forms.TextBox();
            this.menuStrip1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picLeftImage)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picOutput1)).BeginInit();
            this.grpSettings.SuspendLayout();
            this.panelCalibration.SuspendLayout();
            this.SuspendLayout();
            // 
            // menuStrip1
            // 
            this.menuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.fileToolStripMenuItem,
            this.viewToolStripMenuItem,
            this.videoToolStripMenuItem,
            this.toolsToolStripMenuItem});
            this.menuStrip1.Location = new System.Drawing.Point(0, 0);
            this.menuStrip1.Name = "menuStrip1";
            this.menuStrip1.Size = new System.Drawing.Size(898, 24);
            this.menuStrip1.TabIndex = 0;
            this.menuStrip1.Text = "menuStrip1";
            // 
            // fileToolStripMenuItem
            // 
            this.fileToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.exitToolStripMenuItem});
            this.fileToolStripMenuItem.Name = "fileToolStripMenuItem";
            this.fileToolStripMenuItem.Size = new System.Drawing.Size(35, 20);
            this.fileToolStripMenuItem.Text = "File";
            // 
            // exitToolStripMenuItem
            // 
            this.exitToolStripMenuItem.Name = "exitToolStripMenuItem";
            this.exitToolStripMenuItem.Size = new System.Drawing.Size(99, 22);
            this.exitToolStripMenuItem.Text = "Exit";
            this.exitToolStripMenuItem.Click += new System.EventHandler(this.exitToolStripMenuItem_Click);
            // 
            // viewToolStripMenuItem
            // 
            this.viewToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.trackedFeaturesToolStripMenuItem,
            this.searchEllipsesToolStripMenuItem,
            this.overheadMToolStripMenuItem,
            this.particleProbabilitiesToolStripMenuItem,
            this.augmentedRealityToolStripMenuItem});
            this.viewToolStripMenuItem.Name = "viewToolStripMenuItem";
            this.viewToolStripMenuItem.Size = new System.Drawing.Size(45, 20);
            this.viewToolStripMenuItem.Text = "View";
            // 
            // trackedFeaturesToolStripMenuItem
            // 
            this.trackedFeaturesToolStripMenuItem.Name = "trackedFeaturesToolStripMenuItem";
            this.trackedFeaturesToolStripMenuItem.Size = new System.Drawing.Size(177, 22);
            this.trackedFeaturesToolStripMenuItem.Text = "Tracked Features";
            this.trackedFeaturesToolStripMenuItem.Click += new System.EventHandler(this.trackedFeaturesToolStripMenuItem_Click);
            // 
            // searchEllipsesToolStripMenuItem
            // 
            this.searchEllipsesToolStripMenuItem.Name = "searchEllipsesToolStripMenuItem";
            this.searchEllipsesToolStripMenuItem.Size = new System.Drawing.Size(177, 22);
            this.searchEllipsesToolStripMenuItem.Text = "Search Ellipses";
            this.searchEllipsesToolStripMenuItem.Click += new System.EventHandler(this.searchEllipsesToolStripMenuItem_Click);
            // 
            // overheadMToolStripMenuItem
            // 
            this.overheadMToolStripMenuItem.Name = "overheadMToolStripMenuItem";
            this.overheadMToolStripMenuItem.Size = new System.Drawing.Size(177, 22);
            this.overheadMToolStripMenuItem.Text = "Overhead Map";
            this.overheadMToolStripMenuItem.Click += new System.EventHandler(this.overheadMToolStripMenuItem_Click);
            // 
            // particleProbabilitiesToolStripMenuItem
            // 
            this.particleProbabilitiesToolStripMenuItem.Name = "particleProbabilitiesToolStripMenuItem";
            this.particleProbabilitiesToolStripMenuItem.Size = new System.Drawing.Size(177, 22);
            this.particleProbabilitiesToolStripMenuItem.Text = "Particle Probabilities";
            this.particleProbabilitiesToolStripMenuItem.Click += new System.EventHandler(this.particleProbabilitiesToolStripMenuItem_Click);
            // 
            // augmentedRealityToolStripMenuItem
            // 
            this.augmentedRealityToolStripMenuItem.Name = "augmentedRealityToolStripMenuItem";
            this.augmentedRealityToolStripMenuItem.Size = new System.Drawing.Size(177, 22);
            this.augmentedRealityToolStripMenuItem.Text = "Augmented Reality";
            this.augmentedRealityToolStripMenuItem.Click += new System.EventHandler(this.augmentedRealityToolStripMenuItem_Click);
            // 
            // videoToolStripMenuItem
            // 
            this.videoToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.startSentienceToolStripMenuItem,
            this.runSimulationToolStripMenuItem});
            this.videoToolStripMenuItem.Name = "videoToolStripMenuItem";
            this.videoToolStripMenuItem.Size = new System.Drawing.Size(47, 20);
            this.videoToolStripMenuItem.Text = "Video";
            // 
            // startSentienceToolStripMenuItem
            // 
            this.startSentienceToolStripMenuItem.Name = "startSentienceToolStripMenuItem";
            this.startSentienceToolStripMenuItem.Size = new System.Drawing.Size(152, 22);
            this.startSentienceToolStripMenuItem.Text = "Run Camera";
            this.startSentienceToolStripMenuItem.Click += new System.EventHandler(this.startSentienceToolStripMenuItem_Click);
            // 
            // runSimulationToolStripMenuItem
            // 
            this.runSimulationToolStripMenuItem.Name = "runSimulationToolStripMenuItem";
            this.runSimulationToolStripMenuItem.Size = new System.Drawing.Size(152, 22);
            this.runSimulationToolStripMenuItem.Text = "Run Simulation";
            this.runSimulationToolStripMenuItem.Click += new System.EventHandler(this.runSimulationToolStripMenuItem_Click);
            // 
            // toolsToolStripMenuItem
            // 
            this.toolsToolStripMenuItem.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.calibrationToolStripMenuItem,
            this.recordImagesToolStripMenuItem});
            this.toolsToolStripMenuItem.Name = "toolsToolStripMenuItem";
            this.toolsToolStripMenuItem.Size = new System.Drawing.Size(45, 20);
            this.toolsToolStripMenuItem.Text = "Tools";
            // 
            // calibrationToolStripMenuItem
            // 
            this.calibrationToolStripMenuItem.Enabled = false;
            this.calibrationToolStripMenuItem.Name = "calibrationToolStripMenuItem";
            this.calibrationToolStripMenuItem.Size = new System.Drawing.Size(154, 22);
            this.calibrationToolStripMenuItem.Text = "Calibration";
            this.calibrationToolStripMenuItem.Click += new System.EventHandler(this.calibrationToolStripMenuItem_Click);
            // 
            // recordImagesToolStripMenuItem
            // 
            this.recordImagesToolStripMenuItem.Name = "recordImagesToolStripMenuItem";
            this.recordImagesToolStripMenuItem.Size = new System.Drawing.Size(154, 22);
            this.recordImagesToolStripMenuItem.Text = "Record Images";
            this.recordImagesToolStripMenuItem.Click += new System.EventHandler(this.recordImagesToolStripMenuItem_Click);
            // 
            // picLeftImage
            // 
            this.picLeftImage.Location = new System.Drawing.Point(12, 42);
            this.picLeftImage.Name = "picLeftImage";
            this.picLeftImage.Size = new System.Drawing.Size(430, 345);
            this.picLeftImage.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.picLeftImage.TabIndex = 1;
            this.picLeftImage.TabStop = false;
            // 
            // picOutput1
            // 
            this.picOutput1.Location = new System.Drawing.Point(459, 42);
            this.picOutput1.Name = "picOutput1";
            this.picOutput1.Size = new System.Drawing.Size(427, 345);
            this.picOutput1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.picOutput1.TabIndex = 3;
            this.picOutput1.TabStop = false;
            // 
            // grpSettings
            // 
            this.grpSettings.Controls.Add(this.txtCaptureTime);
            this.grpSettings.Controls.Add(this.label11);
            this.grpSettings.Controls.Add(this.lblFrameRateWarning);
            this.grpSettings.Controls.Add(this.lblTracking);
            this.grpSettings.Controls.Add(this.txtProcessingTime);
            this.grpSettings.Controls.Add(this.label10);
            this.grpSettings.Controls.Add(this.cmdBeginTracking);
            this.grpSettings.Controls.Add(this.txtSpeed);
            this.grpSettings.Controls.Add(this.label2);
            this.grpSettings.Controls.Add(this.txtfps);
            this.grpSettings.Controls.Add(this.label1);
            this.grpSettings.Location = new System.Drawing.Point(12, 393);
            this.grpSettings.Name = "grpSettings";
            this.grpSettings.Size = new System.Drawing.Size(875, 65);
            this.grpSettings.TabIndex = 4;
            this.grpSettings.TabStop = false;
            this.grpSettings.Text = "Settings";
            // 
            // txtCaptureTime
            // 
            this.txtCaptureTime.Location = new System.Drawing.Point(643, 22);
            this.txtCaptureTime.Name = "txtCaptureTime";
            this.txtCaptureTime.Size = new System.Drawing.Size(46, 20);
            this.txtCaptureTime.TabIndex = 10;
            this.txtCaptureTime.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(547, 22);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(90, 13);
            this.label11.TabIndex = 9;
            this.label11.Text = "Capture time (mS)";
            // 
            // lblFrameRateWarning
            // 
            this.lblFrameRateWarning.AutoSize = true;
            this.lblFrameRateWarning.BackColor = System.Drawing.Color.Yellow;
            this.lblFrameRateWarning.Font = new System.Drawing.Font("Courier New", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblFrameRateWarning.ForeColor = System.Drawing.Color.Red;
            this.lblFrameRateWarning.Location = new System.Drawing.Point(354, 23);
            this.lblFrameRateWarning.Name = "lblFrameRateWarning";
            this.lblFrameRateWarning.Size = new System.Drawing.Size(164, 22);
            this.lblFrameRateWarning.TabIndex = 8;
            this.lblFrameRateWarning.Text = "Low frame rate";
            this.lblFrameRateWarning.Visible = false;
            // 
            // lblTracking
            // 
            this.lblTracking.AutoSize = true;
            this.lblTracking.BackColor = System.Drawing.Color.Yellow;
            this.lblTracking.Font = new System.Drawing.Font("Courier New", 14.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblTracking.Location = new System.Drawing.Point(384, 23);
            this.lblTracking.Name = "lblTracking";
            this.lblTracking.Size = new System.Drawing.Size(98, 22);
            this.lblTracking.TabIndex = 7;
            this.lblTracking.Text = "Tracking";
            this.lblTracking.Visible = false;
            // 
            // txtProcessingTime
            // 
            this.txtProcessingTime.Location = new System.Drawing.Point(806, 23);
            this.txtProcessingTime.Name = "txtProcessingTime";
            this.txtProcessingTime.Size = new System.Drawing.Size(46, 20);
            this.txtProcessingTime.TabIndex = 6;
            this.txtProcessingTime.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(695, 23);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(105, 13);
            this.label10.TabIndex = 5;
            this.label10.Text = "Processing time (mS)";
            // 
            // cmdBeginTracking
            // 
            this.cmdBeginTracking.Location = new System.Drawing.Point(372, 23);
            this.cmdBeginTracking.Name = "cmdBeginTracking";
            this.cmdBeginTracking.Size = new System.Drawing.Size(110, 19);
            this.cmdBeginTracking.TabIndex = 4;
            this.cmdBeginTracking.Text = "Begin Tracking";
            this.cmdBeginTracking.UseVisualStyleBackColor = true;
            this.cmdBeginTracking.Visible = false;
            this.cmdBeginTracking.Click += new System.EventHandler(this.cmdBeginTracking_Click);
            // 
            // txtSpeed
            // 
            this.txtSpeed.Location = new System.Drawing.Point(260, 23);
            this.txtSpeed.Name = "txtSpeed";
            this.txtSpeed.Size = new System.Drawing.Size(46, 20);
            this.txtSpeed.TabIndex = 3;
            this.txtSpeed.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(189, 23);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(65, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "Speed (m/s)";
            // 
            // txtfps
            // 
            this.txtfps.Location = new System.Drawing.Point(133, 23);
            this.txtfps.Name = "txtfps";
            this.txtfps.Size = new System.Drawing.Size(35, 20);
            this.txtfps.TabIndex = 1;
            this.txtfps.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(16, 23);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(99, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Frames per Second";
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // timUpdate
            // 
            this.timUpdate.Enabled = true;
            this.timUpdate.Interval = 25;
            this.timUpdate.Tick += new System.EventHandler(this.timUpdate_Tick);
            // 
            // panelCalibration
            // 
            this.panelCalibration.Controls.Add(this.cmdCalibrationDone);
            this.panelCalibration.Controls.Add(this.label9);
            this.panelCalibration.Controls.Add(this.txtTargetHeight);
            this.panelCalibration.Controls.Add(this.label8);
            this.panelCalibration.Controls.Add(this.txtTargetWidth);
            this.panelCalibration.Controls.Add(this.label7);
            this.panelCalibration.Controls.Add(this.txtDistToTarget);
            this.panelCalibration.Controls.Add(this.cmdCentreDistortionYdown);
            this.panelCalibration.Controls.Add(this.cmdCentreDistortionYup);
            this.panelCalibration.Controls.Add(this.cmdCentreDistortionXDown);
            this.panelCalibration.Controls.Add(this.cmdCentreDistortionXUp);
            this.panelCalibration.Controls.Add(this.cmdDistortionDown);
            this.panelCalibration.Controls.Add(this.cmdDistortionUp);
            this.panelCalibration.Controls.Add(this.label6);
            this.panelCalibration.Controls.Add(this.txtCentreOfDistortionY);
            this.panelCalibration.Controls.Add(this.label5);
            this.panelCalibration.Controls.Add(this.txtCentreOfDistortionX);
            this.panelCalibration.Controls.Add(this.label4);
            this.panelCalibration.Controls.Add(this.txtDistortion);
            this.panelCalibration.Controls.Add(this.label3);
            this.panelCalibration.Controls.Add(this.txtFOV);
            this.panelCalibration.Location = new System.Drawing.Point(31, 59);
            this.panelCalibration.Name = "panelCalibration";
            this.panelCalibration.Size = new System.Drawing.Size(411, 279);
            this.panelCalibration.TabIndex = 5;
            this.panelCalibration.Visible = false;
            // 
            // cmdCalibrationDone
            // 
            this.cmdCalibrationDone.Location = new System.Drawing.Point(173, 227);
            this.cmdCalibrationDone.Name = "cmdCalibrationDone";
            this.cmdCalibrationDone.Size = new System.Drawing.Size(78, 38);
            this.cmdCalibrationDone.TabIndex = 20;
            this.cmdCalibrationDone.Text = "Done";
            this.cmdCalibrationDone.UseVisualStyleBackColor = true;
            this.cmdCalibrationDone.Click += new System.EventHandler(this.cmdCalibrationDone_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(18, 190);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(95, 13);
            this.label9.TabIndex = 19;
            this.label9.Text = "Target height (mm)";
            // 
            // txtTargetHeight
            // 
            this.txtTargetHeight.Location = new System.Drawing.Point(206, 187);
            this.txtTargetHeight.Name = "txtTargetHeight";
            this.txtTargetHeight.Size = new System.Drawing.Size(45, 20);
            this.txtTargetHeight.TabIndex = 18;
            this.txtTargetHeight.Text = "148.5";
            this.txtTargetHeight.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtTargetHeight.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtTargetHeight_KeyPress);
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(18, 164);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(91, 13);
            this.label8.TabIndex = 17;
            this.label8.Text = "Target width (mm)";
            // 
            // txtTargetWidth
            // 
            this.txtTargetWidth.Location = new System.Drawing.Point(206, 161);
            this.txtTargetWidth.Name = "txtTargetWidth";
            this.txtTargetWidth.Size = new System.Drawing.Size(45, 20);
            this.txtTargetWidth.TabIndex = 16;
            this.txtTargetWidth.Text = "210";
            this.txtTargetWidth.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtTargetWidth.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtTargetWidth_KeyPress);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(18, 138);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(116, 13);
            this.label7.TabIndex = 15;
            this.label7.Text = "Distance to target (mm)";
            // 
            // txtDistToTarget
            // 
            this.txtDistToTarget.Location = new System.Drawing.Point(206, 135);
            this.txtDistToTarget.Name = "txtDistToTarget";
            this.txtDistToTarget.Size = new System.Drawing.Size(45, 20);
            this.txtDistToTarget.TabIndex = 14;
            this.txtDistToTarget.Text = "600";
            this.txtDistToTarget.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtDistToTarget.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtDistToTarget_KeyPress);
            // 
            // cmdCentreDistortionYdown
            // 
            this.cmdCentreDistortionYdown.Location = new System.Drawing.Point(283, 107);
            this.cmdCentreDistortionYdown.Name = "cmdCentreDistortionYdown";
            this.cmdCentreDistortionYdown.Size = new System.Drawing.Size(28, 22);
            this.cmdCentreDistortionYdown.TabIndex = 13;
            this.cmdCentreDistortionYdown.Text = "-";
            this.cmdCentreDistortionYdown.UseVisualStyleBackColor = true;
            this.cmdCentreDistortionYdown.Click += new System.EventHandler(this.cmdCentreDistortionYdown_Click);
            // 
            // cmdCentreDistortionYup
            // 
            this.cmdCentreDistortionYup.Location = new System.Drawing.Point(256, 107);
            this.cmdCentreDistortionYup.Name = "cmdCentreDistortionYup";
            this.cmdCentreDistortionYup.Size = new System.Drawing.Size(28, 22);
            this.cmdCentreDistortionYup.TabIndex = 12;
            this.cmdCentreDistortionYup.Text = "+";
            this.cmdCentreDistortionYup.UseVisualStyleBackColor = true;
            this.cmdCentreDistortionYup.Click += new System.EventHandler(this.cmdCentreDistortionYup_Click);
            // 
            // cmdCentreDistortionXDown
            // 
            this.cmdCentreDistortionXDown.Location = new System.Drawing.Point(283, 81);
            this.cmdCentreDistortionXDown.Name = "cmdCentreDistortionXDown";
            this.cmdCentreDistortionXDown.Size = new System.Drawing.Size(28, 22);
            this.cmdCentreDistortionXDown.TabIndex = 11;
            this.cmdCentreDistortionXDown.Text = "-";
            this.cmdCentreDistortionXDown.UseVisualStyleBackColor = true;
            this.cmdCentreDistortionXDown.Click += new System.EventHandler(this.cmdCentreDistortionXDown_Click);
            // 
            // cmdCentreDistortionXUp
            // 
            this.cmdCentreDistortionXUp.Location = new System.Drawing.Point(256, 81);
            this.cmdCentreDistortionXUp.Name = "cmdCentreDistortionXUp";
            this.cmdCentreDistortionXUp.Size = new System.Drawing.Size(28, 22);
            this.cmdCentreDistortionXUp.TabIndex = 10;
            this.cmdCentreDistortionXUp.Text = "+";
            this.cmdCentreDistortionXUp.UseVisualStyleBackColor = true;
            this.cmdCentreDistortionXUp.Click += new System.EventHandler(this.cmdCentreDistortionXUp_Click);
            // 
            // cmdDistortionDown
            // 
            this.cmdDistortionDown.Location = new System.Drawing.Point(283, 55);
            this.cmdDistortionDown.Name = "cmdDistortionDown";
            this.cmdDistortionDown.Size = new System.Drawing.Size(28, 22);
            this.cmdDistortionDown.TabIndex = 9;
            this.cmdDistortionDown.Text = "-";
            this.cmdDistortionDown.UseVisualStyleBackColor = true;
            this.cmdDistortionDown.Click += new System.EventHandler(this.cmdDistortionDown_Click);
            // 
            // cmdDistortionUp
            // 
            this.cmdDistortionUp.Location = new System.Drawing.Point(256, 55);
            this.cmdDistortionUp.Name = "cmdDistortionUp";
            this.cmdDistortionUp.Size = new System.Drawing.Size(28, 22);
            this.cmdDistortionUp.TabIndex = 8;
            this.cmdDistortionUp.Text = "+";
            this.cmdDistortionUp.UseVisualStyleBackColor = true;
            this.cmdDistortionUp.Click += new System.EventHandler(this.cmdDistortionUp_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(18, 110);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(134, 13);
            this.label6.TabIndex = 7;
            this.label6.Text = "Centre of Distortion vertical";
            // 
            // txtCentreOfDistortionY
            // 
            this.txtCentreOfDistortionY.Location = new System.Drawing.Point(206, 107);
            this.txtCentreOfDistortionY.Name = "txtCentreOfDistortionY";
            this.txtCentreOfDistortionY.Size = new System.Drawing.Size(45, 20);
            this.txtCentreOfDistortionY.TabIndex = 6;
            this.txtCentreOfDistortionY.Text = "125";
            this.txtCentreOfDistortionY.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtCentreOfDistortionY.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtCentreOfDistortionY_KeyPress);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(18, 84);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(145, 13);
            this.label5.TabIndex = 5;
            this.label5.Text = "Centre of Distortion horizontal";
            // 
            // txtCentreOfDistortionX
            // 
            this.txtCentreOfDistortionX.Location = new System.Drawing.Point(206, 81);
            this.txtCentreOfDistortionX.Name = "txtCentreOfDistortionX";
            this.txtCentreOfDistortionX.Size = new System.Drawing.Size(45, 20);
            this.txtCentreOfDistortionX.TabIndex = 4;
            this.txtCentreOfDistortionX.Text = "162";
            this.txtCentreOfDistortionX.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtCentreOfDistortionX.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtCentreOfDistortionX_KeyPress);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(18, 58);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(77, 13);
            this.label4.TabIndex = 3;
            this.label4.Text = "Lens Distortion";
            // 
            // txtDistortion
            // 
            this.txtDistortion.Location = new System.Drawing.Point(206, 55);
            this.txtDistortion.Name = "txtDistortion";
            this.txtDistortion.Size = new System.Drawing.Size(45, 20);
            this.txtDistortion.TabIndex = 2;
            this.txtDistortion.Text = "195";
            this.txtDistortion.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtDistortion.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtDistortion_KeyPress);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(18, 34);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(166, 13);
            this.label3.TabIndex = 1;
            this.label3.Text = "Field of vision (horizontal degrees)";
            // 
            // txtFOV
            // 
            this.txtFOV.Location = new System.Drawing.Point(206, 31);
            this.txtFOV.Name = "txtFOV";
            this.txtFOV.Size = new System.Drawing.Size(45, 20);
            this.txtFOV.TabIndex = 0;
            this.txtFOV.Text = "40";
            this.txtFOV.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.txtFOV.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.txtFOV_KeyPress);
            // 
            // frmMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(898, 468);
            this.Controls.Add(this.panelCalibration);
            this.Controls.Add(this.grpSettings);
            this.Controls.Add(this.picOutput1);
            this.Controls.Add(this.picLeftImage);
            this.Controls.Add(this.menuStrip1);
            this.MainMenuStrip = this.menuStrip1;
            this.Name = "frmMain";
            this.Text = "monoSLAM";
            this.Load += new System.EventHandler(this.frmMain_Load);
            this.menuStrip1.ResumeLayout(false);
            this.menuStrip1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picLeftImage)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picOutput1)).EndInit();
            this.grpSettings.ResumeLayout(false);
            this.grpSettings.PerformLayout();
            this.panelCalibration.ResumeLayout(false);
            this.panelCalibration.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.MenuStrip menuStrip1;
        private System.Windows.Forms.ToolStripMenuItem fileToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem viewToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem videoToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem toolsToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem exitToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem startSentienceToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem calibrationToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem recordImagesToolStripMenuItem;
        private System.Windows.Forms.PictureBox picLeftImage;
        private System.Windows.Forms.PictureBox picOutput1;
        private System.Windows.Forms.GroupBox grpSettings;
        private System.Windows.Forms.TextBox txtfps;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Timer timUpdate;
        private System.Windows.Forms.TextBox txtSpeed;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ToolStripMenuItem trackedFeaturesToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem overheadMToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem searchEllipsesToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem particleProbabilitiesToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem runSimulationToolStripMenuItem;
        private System.Windows.Forms.Panel panelCalibration;
        private System.Windows.Forms.TextBox txtFOV;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox txtCentreOfDistortionY;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox txtCentreOfDistortionX;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txtDistortion;
        private System.Windows.Forms.Button cmdDistortionDown;
        private System.Windows.Forms.Button cmdDistortionUp;
        private System.Windows.Forms.Button cmdCentreDistortionYdown;
        private System.Windows.Forms.Button cmdCentreDistortionYup;
        private System.Windows.Forms.Button cmdCentreDistortionXDown;
        private System.Windows.Forms.Button cmdCentreDistortionXUp;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox txtTargetHeight;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox txtTargetWidth;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox txtDistToTarget;
        private System.Windows.Forms.Button cmdCalibrationDone;
        private System.Windows.Forms.Button cmdBeginTracking;
        private System.Windows.Forms.TextBox txtProcessingTime;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label lblTracking;
        private System.Windows.Forms.ToolStripMenuItem augmentedRealityToolStripMenuItem;
        private System.Windows.Forms.Label lblFrameRateWarning;
        private System.Windows.Forms.TextBox txtCaptureTime;
        private System.Windows.Forms.Label label11;
    }
}

