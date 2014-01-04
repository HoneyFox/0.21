using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using Extensions;
using UnityEngine;

public class RendezMe : Part
{
    #region Singleton Part
    public static RendezMe claimingController = null;
    public static Vessel claimingVessel = null;
    public static void claimControl(Vessel vessel, RendezMe controller, bool forced = false)
    {
        try
        {
            // If the vessel is not the current active vessel, it shouldn't be able to claim.
            if (FlightGlobals.ActiveVessel != vessel)
            {
                //Debug.Log("Only active vessel can claim control");
                return;
            }
        }
        catch (Exception)
        {
        }

        try
        {
            // If the controller is not on the same vessel.
            if (RendezMe.claimingVessel != vessel)
            {
                //Debug.Log("The current controller is on another vessel");
                RendezMe.claimingVessel = vessel;
                RendezMe.claimingController = controller;
            }
            else
            {
                // The current controller is another RendezMe on the vessel.
                //Debug.Log("The current controller is on the same vessel");
                if (RendezMe.claimingController != controller && forced == true)
                {
                    RendezMe.claimingController = controller;
                }
            }
        }
        catch (Exception)
        {
        }
    }

    #endregion

    #region UI State

    private bool addedToDrawQueue = false;

    protected Rect WindowPos;

    private Vector2 _scrollPosition = new Vector2(0, 0);

    public enum UIMode
    {
        OFF,
        VESSELS,
        SELECTED,
        RENDEZVOUS,
        ALIGN,
        SYNC,
    }
    
    private UIMode _mode = UIMode.OFF;
    private bool _modeChanged;

    public UIMode Mode
    {
        get { return _mode; }
        set
        {
            if (_mode == value)
                return;

            _mode = value;
            _modeChanged = true;
        }
    }

    private int _selectedFlyMode;

    private const int WindowIDBase = 18420;

    /// <summary>
    /// The selected vessel's location in FlightGlobals.Vessels[LIST].
    /// </summary>
    private int _selectedVesselIndex;

    /// <summary>
    /// Unique Instance ID of selected vessel
    /// </summary>
    private int _selectedVesselInstanceId;

    private LineRenderer line = null;

    private LineRenderer parkLineX = null;
    private LineRenderer parkLineY = null;
    private LineRenderer parkLineZ = null;

    private LineRenderer relVelLine = null;
    private LineRenderer autoPilotDirectionLine = null;

    private MapView mapCamera = null;
    GameObject obj = new GameObject("Line");
    GameObject parkLineXObj = new GameObject("ParkLineX");
    GameObject parkLineYObj = new GameObject("ParkLineY");
    GameObject parkLineZObj = new GameObject("ParkLineZ");
    GameObject relVelLineObj = new GameObject("RelVelLine");
    GameObject autoPilotDirectionLineObj = new GameObject("AutoPilotDirectionLine");

    AutoRendezVel m_autoRendezVel = new AutoRendezVel();

    #endregion

    #region Sync State

    private const int NumberOfPredictedSyncPoints = 4;

    public enum SynchronizationType
    {
        TargetPeriapsis,
        TargetApoapsis,
        ShipPeriapsis,
        ShipApoapsis,
    }

    public SynchronizationType SyncMode = SynchronizationType.TargetPeriapsis;
    private double _minimumPredictedTimeFromTarget;
    private double _rendezvousAnomaly = 180;
    private readonly float[] _shipTimeToRendezvous = new float[8];
    private readonly float[] _targetTimeToRendezvous = new float[8];
    private readonly string[] _syncString = new string[8];
    private int _closestApproachOrbit, _closestTargetOrbit;
    private double _rendezvousRecalculationTimer;
    
    #endregion

    #region Auto Align State

    private bool _autoAlign = false;
    private bool _autoAlignBurnTriggered = false;

    #endregion


    #region Orbit Phaser State

    private bool _autoPhaser = false;

    private enum AutoPhaserState
    {
        // To auto sync:
        // (I believe apo/peri can be swapped through this.)
        // 1. Wait till we hit apoapsis of target.
        Step1WaitForTargetApsis,
        // 2. Burn to match periapsis of target.
        Step2BurnToMatchNextApsis,
        // 3. Wait till we hit periapsis.
        Step3WaitForTargetApsis,
        // 4. Accelerate until one of next N orbits has a rendezvous time match.
        Step4BurnToRendezvous,
        // 5. Wait till we hit that time.
        Step5WaitForRendezvous,
        // 6. Match orbital velocity with target.
        Step6BurnToMatchVelocity
    };

    private AutoPhaserState _autoPhaserState;
    private double _autoPhaserVelocityGoal;

    private bool _autoPhaseBurnComplete = false;
    #endregion

    #region Rendezvous State

    private Vector3 _relativeVelocity;
    private float _relativeInclination;
    private Vector3 _vectorToTarget;
    private float _targetDistance;

    private bool _killRelativeVelocity = false;
    private Vector3 _localRelativeVelocity = Vector3.zero;

    private bool _homeOnRelativePosition = false;
    private Vector3 _localRelativePosition = Vector3.zero;
    private bool _autoAdjustHomeOn = false;
    private float _autoAdjustHomeOnDistance = 20.0f;

    private long tick = 0;

    #endregion

    #region FlyByWire PID Controller State
    public enum Orient
    {
        Off,
        RelativeVelocity,
        RelativeVelocityAway,
        Target,
        TargetAway,
        Normal,
        AntiNormal,
        MatchTarget,
        MatchTargetAway,
        Prograde,
        Retrograde,
        Align,
        UnAlign
    }

    public string[] ControlModeCaptions = new[]
                                      {
                                          "RVel+", "RVel-", " TGT+ ", " TGT- ", "Match+", "Match-", "Align+", "Align-"
                                      };

    public string[] AlignmentCaptions = new[]
                                       {
                                           "NML\n+", "NML\n-"
                                       };

    public Orient PointAt = Orient.Off;
    private bool _flyByWire;
    private Vector3 _tgtFwd;
    private Vector3 _tgtUp;
    private Vector3 _deriv = Vector3.zero;
    private Vector3 _integral = Vector3.zero;
    private Vector3 _headingError = Vector3.zero;
    private Vector3 _prevError = Vector3.zero;
    private Vector3 _prevVector = Vector3.zero;
    private Vector3 _act = Vector3.zero;

    private string _park_offset_x = "0";
    private string _park_offset_y = "20";
    private string _park_offset_z = "0";
    private Vector3 _park_offset = new Vector3(0.0f, 20.0f, 0.0f);

    private string _tgt_yaw = "0";
    private string _tgt_pit = "0";
    private string _tgt_rol = "0";
    private float _tgt_act_yaw = 0.0f;
    private float _tgt_act_pit = 0.0f;
    private float _tgt_act_rol = 0.0f;

    public float Kp = 10.0F;
    public float Ki = 0.0F;
    public float Kd = 60.0F;

    #endregion

    #region User Interface

    private void WindowGUI(int windowID)
    {
        try
        {
            if (RendezMe.claimingVessel == this.vessel && RendezMe.claimingController != this)
            {
                // The current controller is on the same vessel. Try to claim to replace it.
                RendezMe.claimControl(this.vessel, this);
            }
        }
        catch (Exception)
        {
            RendezMe.claimControl(this.vessel, this, true);
        }

        if (RendezMe.claimingController != this)
        {
            // Failed to claim.
            return;
        }

        // Set up the UI style.
        var sty = new GUIStyle(GUI.skin.button);
        sty.normal.textColor = sty.focused.textColor = Color.white;
        sty.hover.textColor = sty.active.textColor = Color.yellow;
        sty.onNormal.textColor = sty.onFocused.textColor = sty.onHover.textColor = sty.onActive.textColor = Color.green;
        sty.padding = new RectOffset(8, 8, 8, 8);

        GUILayout.BeginVertical();

        if (Mode == UIMode.OFF)
            RenderOffUI(sty);

        if (Mode == UIMode.VESSELS)
            RenderVesselsUI(sty);

        if (Mode == UIMode.SELECTED)
            RenderSelectedUI(sty);

        if (Mode == UIMode.ALIGN)
            RenderAlignUI(sty);

        // TIME TO NODES
        // BURN TIMER?
        // DELTA RINC
        // ORIENTATION FOR NEXT BURN (NORMAL/ANTINORMAL)
        // AUTOPILOT(NORMAL/ANTINORMAL)

        if (Mode == UIMode.SYNC)
            RenderSyncUI(sty);

        if (Mode == UIMode.RENDEZVOUS)
            RenderRendezvousUI(sty);

        GUILayout.EndVertical();

        //DragWindow makes the window draggable. The Rect specifies which part of the window it can by dragged by, and is 
        //clipped to the actual boundary of the window. You can also pass no argument at all and then the window can by
        //dragged by any part of it. Make sure the DragWindow command is AFTER all your other GUI input stuff, or else
        //it may "cover up" your controls and make them stop responding to the mouse.
        GUI.DragWindow(new Rect(0, 0, 10000, 20));
    }

    private void RenderOffUI(GUIStyle sty)
    {
        if (GUILayout.Button("OPEN", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.VESSELS;
        }
    }

    private void RenderVesselsUI(GUIStyle sty)
    {
        if (GUILayout.Button("HIDE", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.OFF;
        }
        GUILayout.Box("Select Target");

        //TODO: ADD BODY SUPPORT
        //create a button for each vessel, and store the location of the selected vessel

        _scrollPosition = GUILayout.BeginScrollView(_scrollPosition, GUILayout.Width(300), GUILayout.Height(300));

        // Generate and sort an array of vessels by distance.
        List<Vessel> vesselList = new List<Vessel>(FlightGlobals.Vessels);
        var vdc = new VesselDistanceComparer();
        vdc.OriginVessel = vessel;

        vesselList.Sort(vdc);

        for (int i = 0; i < vesselList.Count; i++)
        {
            // Skip ourselves.
            if (vesselList[i] == vessel)
                continue;

            //if (vesselList[i].LandedOrSplashed)
            //    continue;

            // Skip stuff around other worlds.
            if (vessel.orbit.referenceBody != vesselList[i].orbit.referenceBody)
                continue;

            // Calculate the distance.
            float d = Vector3.Distance(vesselList[i].transform.position, vessel.transform.position);

            if (GUILayout.Button((d / 1000).ToString("F1") + "km " + vesselList[i].vesselName, sty,
                                 GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.SELECTED;
                _selectedVesselInstanceId = vesselList[i].GetInstanceID();
                _selectedVesselIndex = FlightGlobals.Vessels.IndexOf(vesselList[i]);
            }
        }

        GUILayout.EndScrollView();
    }

    private void RenderSelectedUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.VESSELS;
        }

        if (GUILayout.Button((FlightGlobals.Vessels[_selectedVesselIndex].vesselName), sty, GUILayout.ExpandWidth(true)))
        {
            _flyByWire = false;
            Mode = UIMode.VESSELS;
        }
        if (GUILayout.Button("Align Planes", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.ALIGN;
        }
        if (GUILayout.Button("Sync Orbits", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.SYNC;
        }
        if (GUILayout.Button("Rendezvous", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.RENDEZVOUS;
        }

    }

    private void RenderAlignUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.VESSELS;
        }

        if (GUILayout.Button("Align Planes", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.SELECTED;
            _flyByWire = false;
        }

        // FIXME_HF: I don't know why it's not right here.
        if (_relativeInclination < 0.0)
        {
            GUILayout.Box("Time to AN : " +
                          vessel.orbit.GetTimeToRelDN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
            GUILayout.Box("Time to DN : " +
                          vessel.orbit.GetTimeToRelAN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
        }
        else
        {
            GUILayout.Box("Time to AN : " +
                              vessel.orbit.GetTimeToRelAN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
            GUILayout.Box("Time to DN : " +
                          vessel.orbit.GetTimeToRelDN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
        }
        GUILayout.Box("Relative Inclination :" + _relativeInclination.ToString("F2"));

        if(GUILayout.Button(_autoAlign ? "ALIGNING" : "Auto-Align", sty, GUILayout.ExpandWidth(true)))
        {
            _autoAlignBurnTriggered = false;
            _autoAlign = !_autoAlign;
            _modeChanged = true;
        }

        if (_flyByWire == false)
        {
            if (GUILayout.Button("Orbit Normal", sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.Normal;
                _modeChanged = true;
            }

            if (GUILayout.Button("Anti Normal", sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.AntiNormal;
                _modeChanged = true;
            }
        }

        if (_flyByWire && (PointAt == Orient.Normal || PointAt == Orient.AntiNormal))
        {
            if (GUILayout.Button("Disable " + PointAt.ToString(), sty, GUILayout.ExpandWidth(true)))
            {
                FlightInputHandler.SetNeutralControls();
                _flyByWire = false;
                _modeChanged = true;
            }
        }
    }

    private void RenderSyncUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.VESSELS;
        }

        GUILayout.BeginVertical();
        if (GUILayout.Button("Sync Orbits", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.SELECTED;
            _flyByWire = false;
        }
        
        for (int i = 0; i < NumberOfPredictedSyncPoints; i++)
        {
            if (i != (int) SyncMode) 
                continue;

            if (GUILayout.Button("Mode", sty, GUILayout.ExpandWidth(true)))
            {
                if (i == NumberOfPredictedSyncPoints - 1) SyncMode = 0;
                else SyncMode = SyncMode + 1;
            }
            GUILayout.Box(SyncMode.ToString(), GUILayout.ExpandWidth(true));
        }
        
        GUILayout.Box("Orbit		ShipToR		TgtToR ", sty, GUILayout.ExpandWidth(true));
        for (int i = 0; i < 8; i++)
            GUILayout.Box(_syncString[i]);

        GUILayout.Box("Closest Approach : Orbit " + _closestApproachOrbit.ToString() + " | " + _closestTargetOrbit.ToString());
        GUILayout.Box("Min Separation (sec) : " + _minimumPredictedTimeFromTarget.ToString("f1"));
        
        /*
        if(GUILayout.Button(_autoPhaser ? _autoPhaserState.ToString() : "Auto Sync", sty, GUILayout.ExpandWidth(true)))
        {
            _autoPhaser = !_autoPhaser;
            _autoPhaserState = AutoPhaserState.Step1WaitForTargetApsis;
        }
         * */

        GUILayout.EndVertical();
    }

    private void RenderRendezvousUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            m_autoRendezVel.m_rendezState = AutoRendezVel.RendezState.Invalid;
            _homeOnRelativePosition = false;
            _killRelativeVelocity = false;
            Mode = UIMode.VESSELS;
            return;
        }

        Vessel selectedVessel = null;
        try
        {
            selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;
        }
        catch (Exception)
        {
            _flyByWire = false;
            m_autoRendezVel.m_rendezState = AutoRendezVel.RendezState.Invalid;
            _homeOnRelativePosition = false;
            _killRelativeVelocity = false;
            Mode = UIMode.VESSELS;
            return;
        }

        //the above check should prevent a crash when the vessel we are looking for is destroyed
        //learn how to use list.exists etc...
        if (GUILayout.Button(selectedVessel.vesselName, sty, GUILayout.ExpandWidth(true)))
        {
            _flyByWire = false;
            m_autoRendezVel.m_rendezState = AutoRendezVel.RendezState.Invalid;
            _homeOnRelativePosition = false;
            _killRelativeVelocity = false;
            Mode = UIMode.SELECTED;
        }

        GUILayout.Box("Distance: " + _targetDistance.ToString("F1"), GUILayout.Width(300));
        GUILayout.Box("Rel Inc : " + _relativeInclination.ToString("F3"));
        GUILayout.Box("Rel VelM: " + _relativeVelocity.magnitude.ToString("F2"));

        // Take the relative velocity and project into ship local space.
        _localRelativeVelocity = vessel.transform.worldToLocalMatrix.MultiplyVector(_relativeVelocity);
        _localRelativePosition = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.position);
        
        // Calculate the LOS rate here.
        Vector3 _projectedRelativeVelocity;
        Vector3 _relativePosition = selectedVessel.transform.position - vessel.transform.position;
        float _closureRate = Vector3.Dot(_relativePosition, _relativeVelocity) / _relativePosition.magnitude;
        Vector3 _projectedClosureVector = new Vector3(_relativePosition.x, _relativePosition.y, _relativePosition.z);
        _projectedClosureVector.Normalize();
        _projectedClosureVector *= _closureRate;
        _projectedRelativeVelocity = _relativeVelocity - _projectedClosureVector;

        if(GUILayout.Button(_killRelativeVelocity == false ? "Intercept" : "FIRING", sty, GUILayout.ExpandWidth(true)))
            _killRelativeVelocity = !_killRelativeVelocity;

        if (GUILayout.Button(_homeOnRelativePosition == false ? "Home on" : "HOMING", sty, GUILayout.ExpandWidth(true)))
        {
            _homeOnRelativePosition = !_homeOnRelativePosition;
            if(_homeOnRelativePosition == false)
                _modeChanged = true;
        }

        if (_homeOnRelativePosition)
        {
            if(_autoAdjustHomeOn == false)
            {
                GUILayout.BeginHorizontal();
                _park_offset_x = GUILayout.TextField(_park_offset_x, GUILayout.Width(50));
                _park_offset_x = Regex.Replace(_park_offset_x, @"[^\d+-.]", "");
                _park_offset_y = GUILayout.TextField(_park_offset_y, GUILayout.Width(50));
                _park_offset_y = Regex.Replace(_park_offset_y, @"[^\d+-.]", "");
                _park_offset_z = GUILayout.TextField(_park_offset_z, GUILayout.Width(50));
                _park_offset_z = Regex.Replace(_park_offset_z, @"[^\d+-.]", "");
                if (GUILayout.Button("Set", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                {
                    _park_offset.x = Convert.ToSingle(_park_offset_x);
                    _park_offset.y = Convert.ToSingle(_park_offset_y);
                    _park_offset.z = Convert.ToSingle(_park_offset_z);
                    GUIUtility.keyboardControl = 0;
                }
                if (GUILayout.Button("Reset", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                {
                    _park_offset_x = "0"; _park_offset_y = "20"; _park_offset_z = "0";
                    _park_offset.x = 0.0f; _park_offset.y = 20.0f; _park_offset.z = 0.0f;
                    GUIUtility.keyboardControl = 0;
                }
                if (GUILayout.Button("Auto", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                {
                    if (FlightGlobals.fetch != null)
                    {
                        if (FlightGlobals.fetch.VesselTarget is ModuleDockingNode)
                        {
                            _autoAdjustHomeOn = true;
                            _autoAdjustHomeOnDistance = 20.0f;
                        }
                    }
                }
                GUILayout.EndHorizontal();
            }
            else
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.RepeatButton("--", GUILayout.Width(30.0f), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOnDistance -= 0.5f;
                    if (_autoAdjustHomeOnDistance < 0.0f) _autoAdjustHomeOnDistance = 0.0f;
                }
                if (GUILayout.RepeatButton("-", GUILayout.Width(20.0f), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOnDistance -= 0.05f;
                    if (_autoAdjustHomeOnDistance < 0.0f) _autoAdjustHomeOnDistance = 0.0f;
                }
                GUILayout.Button(_autoAdjustHomeOnDistance.ToString("F2"), GUILayout.Width(70.0f), GUILayout.Height(30.0f));
                if (GUILayout.RepeatButton("+", GUILayout.Width(20.0f), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOnDistance += 0.05f;
                    if (_autoAdjustHomeOnDistance > 50.0f) _autoAdjustHomeOnDistance = 50.0f;
                }
                if (GUILayout.RepeatButton("++", GUILayout.Width(30.0f), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOnDistance += 0.5f;
                    if (_autoAdjustHomeOnDistance > 50.0f) _autoAdjustHomeOnDistance = 50.0f;
                }
                if (GUILayout.Button("Reset", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOnDistance = 20.0f;
                }
                if (GUILayout.Button("Manual", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                {
                    _autoAdjustHomeOn = false;
                }
                if (FlightGlobals.fetch != null)
                {
                    if (FlightGlobals.fetch.VesselTarget is ModuleDockingNode)
                    {
                        ModuleDockingNode dockingNode = FlightGlobals.fetch.VesselTarget as ModuleDockingNode;
                        Vector3 dockLocalPosition = dockingNode.vessel.transform.InverseTransformPoint(dockingNode.nodeTransform.position);
                        Vector3 dockLocalRotation = dockingNode.vessel.transform.InverseTransformDirection(dockingNode.nodeTransform.forward);
                        
                        float safetyDistance = _autoAdjustHomeOnDistance;
                        Vector3 autoParkPosition = dockLocalPosition + dockLocalRotation * safetyDistance;

                        Vector3 ownDockPortFix = dockingNode.vessel.transform.InverseTransformDirection(this.vessel.transform.position - this.vessel.ReferenceTransform.position);
                        //Debug.Log("OwnDockPortFix: " + ownDockPortFix.ToString());

                        autoParkPosition += ownDockPortFix;

                        _park_offset_x = autoParkPosition.x.ToString("F2");
                        _park_offset_y = autoParkPosition.y.ToString("F2");
                        _park_offset_z = autoParkPosition.z.ToString("F2");

                        _park_offset.x = Convert.ToSingle(_park_offset_x);
                        _park_offset.y = Convert.ToSingle(_park_offset_y);
                        _park_offset.z = Convert.ToSingle(_park_offset_z);
                        //GUIUtility.keyboardControl = 0;
                    }
                    else
                    {
                        _autoAdjustHomeOn = false;
                    }
                }
                else
                {
                    _autoAdjustHomeOn = false;
                }
                GUILayout.EndHorizontal();
            }
        }

        GUILayout.Box("Rel Vel : " + _localRelativeVelocity.x.ToString("F2") + ", " + _localRelativeVelocity.y.ToString("F2") + ", " + _localRelativeVelocity.z.ToString("F2"));
        GUILayout.Box("LOS Vel : " + _projectedRelativeVelocity.x.ToString("F2") + ", " + _projectedRelativeVelocity.y.ToString("F2") + ", " + _projectedRelativeVelocity.z.ToString("F2"));
        GUILayout.Box("Rel Pos : " + _localRelativePosition.x.ToString("F2") + ", " + _localRelativePosition.y.ToString("F2") + ", " + _localRelativePosition.z.ToString("F2"));

        if (selectedVessel.mainBody == vessel.mainBody)
        {
            Vector3d tgtPos = selectedVessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime());
            Vector3d selfPos = vessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime());
            
            //PE
            double timeDiff = 0.0;
            
            Vector3 ownEccVec = vessel.orbit.eccVec.normalized;
            Vector3 tgtEccVec = selectedVessel.orbit.eccVec.normalized;
            float dot = Vector3.Dot(ownEccVec, tgtEccVec);
            
            if (vessel.orbit.eccentricity < 0.1 && selectedVessel.orbit.eccentricity < 0.1)
            {
                // Both in circular orbits.
                selfPos = Vector3d.Project(new Vector3d(selfPos.x, 0, selfPos.z), selfPos);
                tgtPos = Vector3d.Project(new Vector3d(tgtPos.x, 0, tgtPos.z), tgtPos);
                Vector3d prograde = new Vector3d();
                prograde = Quaternion.AngleAxis(90, Vector3d.forward) * selfPos;
                double phaseDiff = Vector3d.Angle(selfPos, tgtPos);
                if (Vector3d.Angle(prograde, tgtPos) > 90)
                    phaseDiff = 360 - phaseDiff;

                phaseDiff = (phaseDiff + 360) % 360;
                if (phaseDiff > 180) phaseDiff -= 360;
                timeDiff = selectedVessel.orbit.period * phaseDiff / 360.0;

                //Debug.Log("Two Circular Orbits: DT: " + timeDiff.ToString("F1"));
            }
            else
            {
                if (selectedVessel.orbit.eccentricity < 0.1)
                {
                    // Target is in circular orbit. Self is not.
                    double ownTimeToPe = vessel.orbit.timeToPe;
                    if (vessel.orbit.timeToPe > vessel.orbit.period / 2)
                        ownTimeToPe = vessel.orbit.timeToPe - vessel.orbit.period;
                    //Debug.Log("Own TTP: " + ownTimeToPe.ToString("F1"));

                    Vector3 selfPosAtPe = vessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime() + vessel.orbit.timeToPe);
                    selfPosAtPe = Vector3d.Project(new Vector3d(selfPosAtPe.x, 0, selfPosAtPe.z), selfPosAtPe);
                    Vector3 tgtPosWhenSelfAtPe = selectedVessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime() + vessel.orbit.timeToPe);
                    tgtPosWhenSelfAtPe = Vector3d.Project(new Vector3d(tgtPosWhenSelfAtPe.x, 0, tgtPosWhenSelfAtPe.z), tgtPosWhenSelfAtPe);
                    Vector3d prograde = new Vector3d();
                    prograde = Quaternion.AngleAxis(90, Vector3d.forward) * selfPosAtPe;
                    double phaseDiff = Vector3d.Angle(selfPosAtPe, tgtPosWhenSelfAtPe);
                    if (Vector3d.Angle(prograde, tgtPosWhenSelfAtPe) > 90)
                        phaseDiff = 360 - phaseDiff;

                    phaseDiff = (phaseDiff + 360) % 360;
                    if (phaseDiff > 180) phaseDiff -= 360;
                    timeDiff = selectedVessel.orbit.period * phaseDiff / 360.0;

                    //Debug.Log("Target Circular Own Elliptic: DT: " + timeDiff.ToString("F1"));
                }
                else
                {
                    // Target has elliptic orbit. Self has circular orbit.
                    // Or both have elliptic orbits. Their eccVectors should match+/- to each other.
                    double ownTimeToPe = vessel.orbit.timeToPe;
                    if (vessel.orbit.timeToPe > vessel.orbit.period / 2)
                        ownTimeToPe = vessel.orbit.timeToPe - vessel.orbit.period;
                    //Debug.Log("Own TTP: " + ownTimeToPe.ToString("F1"));
                    double tgtTimeToPe = selectedVessel.orbit.timeToPe;
                    if (selectedVessel.orbit.timeToPe > selectedVessel.orbit.period / 2)
                        tgtTimeToPe = selectedVessel.orbit.timeToPe - selectedVessel.orbit.period;
                    //Debug.Log("Tgt TTP: " + tgtTimeToPe.ToString("F1"));
                    double tgtTimeToAp = selectedVessel.orbit.timeToAp;
                    if (selectedVessel.orbit.timeToAp > selectedVessel.orbit.period / 2)
                        tgtTimeToAp = selectedVessel.orbit.timeToAp - selectedVessel.orbit.period;
                    //Debug.Log("Tgt TTA: " + tgtTimeToAp.ToString("F1"));

                    if (dot > 0)
                        timeDiff = ownTimeToPe - tgtTimeToPe;
                    else
                        timeDiff = ownTimeToPe - tgtTimeToAp;
                    //Debug.Log("Elliptic Orbit: DT: " + timeDiff.ToString("F1"));
                }
            }

            int k = 0;
            double newOwnPeriod = 0;
            if (selectedVessel.orbit.period < vessel.orbit.period)
            {
                // Our orbit is slower.
                k = (int)Math.Round((vessel.orbit.period + timeDiff) / selectedVessel.orbit.period);
                k = Math.Max(1, k);
                if (k > 1) k--;
                newOwnPeriod = k * selectedVessel.orbit.period - timeDiff;
            }
            else
            {
                // Our orbit is faster.
                k = (int)Math.Round((selectedVessel.orbit.period - timeDiff) / vessel.orbit.period);
                k = Math.Max(1, k);
                if (k > 1) k--;
                newOwnPeriod = (selectedVessel.orbit.period - timeDiff) / k;
            }

            //GUILayout.Label("Own Period: " + vessel.orbit.period.ToString("F1") + " Tgt Period: " + selectedVessel.orbit.period.ToString("F1"));
            //GUILayout.Label("New Period adjusted to: k=" + k.ToString() + ", T=" + newOwnPeriod.ToString("F1"), GUILayout.ExpandWidth(true));
            
            double miu = selectedVessel.mainBody.gravParameter;
            double h = vessel.orbit.PeA + vessel.mainBody.Radius;

            double vNewPe = Math.Sqrt
            (
                2 * miu / h - miu /
                (
                    Math.Pow
                    (
                        miu * Math.Pow
                        (
                            newOwnPeriod, 2.0
                        ) / 4.0 / Math.PI / Math.PI, 1.0 / 3.0
                    )
                )
            );

            double vOld = vessel.orbit.vel.magnitude;

            // AP
            timeDiff = 0.0;
            k = 0;
            newOwnPeriod = 0;

            if (vessel.orbit.eccentricity < 0.1 && selectedVessel.orbit.eccentricity < 0.1)
            {
                // Both in circular orbits.
                selfPos = Vector3d.Project(new Vector3d(selfPos.x, 0, selfPos.z), selfPos);
                tgtPos = Vector3d.Project(new Vector3d(tgtPos.x, 0, tgtPos.z), tgtPos);
                Vector3d prograde = new Vector3d();
                prograde = Quaternion.AngleAxis(90, Vector3d.forward) * selfPos;
                double phaseDiff = Vector3d.Angle(selfPos, tgtPos);
                if (Vector3d.Angle(prograde, tgtPos) > 90)
                    phaseDiff = 360 - phaseDiff;

                phaseDiff = (phaseDiff + 360) % 360;
                if (phaseDiff > 180) phaseDiff -= 360;
                timeDiff = selectedVessel.orbit.period * phaseDiff / 360.0;
                
                //Debug.Log("Two Circular Orbits: DT: " + timeDiff.ToString("F1"));
            }
            else
            {
                if (selectedVessel.orbit.eccentricity < 0.1)
                {
                    // Target is in circular orbit. Self is not.
                    double ownTimeToAp = vessel.orbit.timeToAp;
                    if (vessel.orbit.timeToAp > vessel.orbit.period / 2)
                        ownTimeToAp = vessel.orbit.timeToAp - vessel.orbit.period;
                    //Debug.Log("Own TTA: " + ownTimeToAp.ToString("F1"));

                    Vector3 selfPosAtAp = vessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime() + vessel.orbit.timeToAp);
                    selfPosAtAp = Vector3d.Project(new Vector3d(selfPosAtAp.x, 0, selfPosAtAp.z), selfPosAtAp);
                    Vector3 tgtPosWhenSelfAtAp = selectedVessel.orbit.getRelativePositionAtUT(Planetarium.GetUniversalTime() + vessel.orbit.timeToAp);
                    tgtPosWhenSelfAtAp = Vector3d.Project(new Vector3d(tgtPosWhenSelfAtAp.x, 0, tgtPosWhenSelfAtAp.z), tgtPosWhenSelfAtAp);
                    Vector3d prograde = new Vector3d();
                    prograde = Quaternion.AngleAxis(90, Vector3d.forward) * selfPosAtAp;
                    double phaseDiff = Vector3d.Angle(selfPosAtAp, tgtPosWhenSelfAtAp);
                    if (Vector3d.Angle(prograde, tgtPosWhenSelfAtAp) > 90)
                        phaseDiff = 360 - phaseDiff;

                    phaseDiff = (phaseDiff + 360) % 360;
                    if (phaseDiff > 180) phaseDiff -= 360;
                    timeDiff = selectedVessel.orbit.period * phaseDiff / 360.0;

                    //Debug.Log("Target Circular Own Elliptic: DT: " + timeDiff.ToString("F1"));
                }
                else
                {
                    // Target has elliptic orbit. Self has circular orbit.
                    // Or both have elliptic orbits. Their eccVectors should match+/- to each other.
                    double ownTimeToAp = vessel.orbit.timeToAp;
                    if (vessel.orbit.timeToAp > vessel.orbit.period / 2)
                        ownTimeToAp = vessel.orbit.timeToAp - vessel.orbit.period;
                    //Debug.Log("Own TTA: " + ownTimeToAp.ToString("F1"));
                    double tgtTimeToAp = selectedVessel.orbit.timeToAp;
                    if (selectedVessel.orbit.timeToAp > selectedVessel.orbit.period / 2)
                        tgtTimeToAp = selectedVessel.orbit.timeToAp - selectedVessel.orbit.period;
                    //Debug.Log("Tgt TTA: " + tgtTimeToAp.ToString("F1"));
                    double tgtTimeToPe = selectedVessel.orbit.timeToPe;
                    if (selectedVessel.orbit.timeToPe > selectedVessel.orbit.period / 2)
                        tgtTimeToPe = selectedVessel.orbit.timeToPe - selectedVessel.orbit.period;
                    //Debug.Log("Tgt TTP: " + tgtTimeToPe.ToString("F1"));

                    if (dot > 0)
                        timeDiff = ownTimeToAp - tgtTimeToAp;
                    else
                        timeDiff = ownTimeToAp - tgtTimeToPe;
                    //Debug.Log("Elliptic Orbit: DT: " + timeDiff.ToString("F1"));
                }
            }

            if (selectedVessel.orbit.period < vessel.orbit.period)
            {
                // Our orbit is slower.
                k = (int)Math.Round((vessel.orbit.period + timeDiff) / selectedVessel.orbit.period);
                k = Math.Max(1, k);
                if (k > 1) k--;
                newOwnPeriod = k * selectedVessel.orbit.period - timeDiff;
            }
            else
            {
                // Our orbit is faster.
                k = (int)Math.Round((selectedVessel.orbit.period - timeDiff) / vessel.orbit.period);
                k = Math.Max(1, k);
                if (k > 1) k--;
                newOwnPeriod = (selectedVessel.orbit.period - timeDiff) / k;
            }

            //GUILayout.Label("Own Period: " + vessel.orbit.period.ToString("F1") + " Tgt Period: " + selectedVessel.orbit.period.ToString("F1"));
            //GUILayout.Label("New Period adjusted to: k=" + k.ToString() + ", T=" + newOwnPeriod.ToString("F1"), GUILayout.ExpandWidth(true));
            
            h = vessel.orbit.ApA + vessel.mainBody.Radius;
            
            double vNewAp = Math.Sqrt
            (
                2 * miu / h - miu /
                (
                    Math.Pow
                    (
                        miu * Math.Pow
                        (
                            newOwnPeriod, 2.0
                        ) / 4.0 / Math.PI / Math.PI, 1.0 / 3.0
                    )
                )
            );

            GUILayout.Box("Rendez Vel : @Ap " + vNewAp.ToString("F1") + ", @Pe " + vNewPe.ToString("F1") + " / " + vOld.ToString("F1"));
            m_autoRendezVel.setVelocities(vNewAp, vNewPe, vOld);
            if (_flyByWire == false || m_autoRendezVel.m_rendezState != AutoRendezVel.RendezState.Invalid)
            {
                m_autoRendezVel.drawGUI(ref _flyByWire, ref _modeChanged);
            }
        }
        else
        {
            GUILayout.Box("Rendez Vel : Available When Orbitting the Same Body.");
        }

        if (_flyByWire == false)
        {
            GUILayout.BeginHorizontal();

            if (GUILayout.Button(ControlModeCaptions[0], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.RelativeVelocity;
                _modeChanged = true;
                _selectedFlyMode = 0;
            }


            if (GUILayout.Button(ControlModeCaptions[1], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.RelativeVelocityAway;
                _modeChanged = true;
                _selectedFlyMode = 1;
            }


            if (GUILayout.Button(ControlModeCaptions[2], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.Target;
                _modeChanged = true;
                _selectedFlyMode = 2;
            }

            if (GUILayout.Button(ControlModeCaptions[3], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.TargetAway;
                _modeChanged = true;
                _selectedFlyMode = 3;
            }

            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();

            if (GUILayout.Button(ControlModeCaptions[4], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.MatchTarget;
                _modeChanged = true;
                _selectedFlyMode = 4;
            }

            if (GUILayout.Button(ControlModeCaptions[5], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.MatchTargetAway;
                _modeChanged = true;
                _selectedFlyMode = 5;
            }


            if (GUILayout.Button(ControlModeCaptions[6], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.Align;
                _modeChanged = true;
                _selectedFlyMode = 6;
            }


            if (GUILayout.Button(ControlModeCaptions[7], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.UnAlign;
                _modeChanged = true;
                _selectedFlyMode = 7;
            }

            GUILayout.EndHorizontal();

        }

        if (_flyByWire)
        {
            if (m_autoRendezVel.m_rendezState == AutoRendezVel.RendezState.Invalid)
            {
                if (_selectedFlyMode >= 4 && _selectedFlyMode <= 5)
                {
                    GUILayout.BeginHorizontal(GUILayout.ExpandWidth(true));
                    GUILayout.Label("YPR", GUILayout.Width(30));
                    _tgt_yaw = GUILayout.TextField(_tgt_yaw, GUILayout.Width(40));
                    _tgt_yaw = Regex.Replace(_tgt_yaw, @"[^\d+-.]", "");
                    _tgt_pit = GUILayout.TextField(_tgt_pit, GUILayout.Width(40));
                    _tgt_pit = Regex.Replace(_tgt_pit, @"[^\d+-.]", "");
                    _tgt_rol = GUILayout.TextField(_tgt_rol, GUILayout.Width(40));
                    _tgt_rol = Regex.Replace(_tgt_rol, @"[^\d+-.]", "");
                    if (GUILayout.Button("Set", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                    {
                        _tgt_act_yaw = Convert.ToSingle(_tgt_yaw);
                        _tgt_act_pit = Convert.ToSingle(_tgt_pit);
                        _tgt_act_rol = Convert.ToSingle(_tgt_rol);
                        GUIUtility.keyboardControl = 0;
                    }
                    if (GUILayout.Button("Reset", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                    {
                        _tgt_yaw = "0";
                        _tgt_pit = "0";
                        _tgt_rol = "0";
                        _tgt_act_yaw = 0.0f;
                        _tgt_act_pit = 0.0f;
                        _tgt_act_rol = 0.0f;
                        GUIUtility.keyboardControl = 0;
                    }
                    if (GUILayout.Button("Auto", GUILayout.ExpandWidth(true), GUILayout.Height(30.0f)))
                    {
                        if (FlightGlobals.fetch != null)
                        {
                            if (FlightGlobals.fetch.VesselTarget is ModuleDockingNode)
                            {
                                ModuleDockingNode targetDockingNode = FlightGlobals.fetch.VesselTarget as ModuleDockingNode;
                                Quaternion targetOrientation = targetDockingNode.vessel.transform.rotation;
                                Quaternion targetDockOrientation = Quaternion.LookRotation(-targetDockingNode.nodeTransform.up, targetDockingNode.nodeTransform.forward);
                                Quaternion ownOrientation = this.vessel.transform.rotation;
                                Quaternion ownDockOrientation = this.vessel.ReferenceTransform.rotation;

                                // This rotation should make own dock port facing forwards.
                                // ownRotFix = ownOrientation <- ownDockOrientation
                                Debug.Log("ownOrientation: " + ownOrientation.eulerAngles.ToString());
                                Debug.Log("ownDockOrientation: " + ownDockOrientation.eulerAngles.ToString());
                                Quaternion ownRotFix = Quaternion.Inverse(ownDockOrientation) * ownOrientation;
                                Debug.Log("OwnRotFix: " + ownRotFix.eulerAngles.ToString());
                                // This rotation represents the rotation from target orientation to target dock orientation.
                                // targetDockRotation = targetDockOrientation <- targetOrientation
                                Debug.Log("targetOrientation: " + targetOrientation.eulerAngles.ToString());
                                Debug.Log("targetDockOrientation: " + targetDockOrientation.eulerAngles.ToString());
                                Quaternion targetDockRotation = Quaternion.Inverse(targetOrientation) * targetDockOrientation;
                                Debug.Log("TargetDockRot: " + targetDockRotation.eulerAngles.ToString());
                                // This rotation represents the entire rotation.
                                Quaternion finalRot = Quaternion.identity;
                                if (_selectedFlyMode == 4)
                                {
                                    finalRot = ownRotFix * Quaternion.Euler(0.0f, 0.0f, 180.0f) * Quaternion.Inverse(targetDockRotation) * Quaternion.Euler(0.0f, 0.0f, 180.0f);
                                    // from right to left:
                                    // Rotate back to Match- mode, use the same way to rotate to the target, but without own dock rotation fix.
                                    // Now we have vessel orientation pointing towards the target.
                                    // Rotate for 180 degrees, then apply the own dock rotation fix.
                                }
                                else if (_selectedFlyMode == 5)
                                {
                                    finalRot = ownRotFix * Quaternion.Inverse(targetDockRotation);
                                }
                                Debug.Log("FinalRot: " + finalRot.eulerAngles.ToString());

                                int y = 0, p = 0, r = 0;
                                y = Mathf.RoundToInt(finalRot.eulerAngles.z);
                                p = Mathf.RoundToInt(-finalRot.eulerAngles.x);
                                r = Mathf.RoundToInt(-finalRot.eulerAngles.y);
                                
                                while(y > 180) y -= 360; while(y <= -180) y += 360;
                                while (p > 180) p -= 360; while (p <= -180) p += 360;
                                while (r > 180) r -= 360; while (r <= -180) r += 360;
                                _tgt_yaw = y.ToString();
                                _tgt_pit = p.ToString();
                                _tgt_rol = r.ToString();
                                _tgt_act_yaw = y;
                                _tgt_act_pit = p;
                                _tgt_act_rol = r;
                            }
                        }
                    }
                    GUILayout.EndHorizontal();
                }
            }

            if (m_autoRendezVel.m_rendezState == AutoRendezVel.RendezState.Invalid)
            {
                // It's impossible to show this when the autoRendezVel is active.
                if (GUILayout.Button("Disable " + ControlModeCaptions[_selectedFlyMode].Trim(), sty, GUILayout.ExpandWidth(true)))
                {
                    FlightInputHandler.SetNeutralControls();
                    _flyByWire = false;
                    _modeChanged = true;
                }
            }
        }
    }

    /// <summary>
    /// Draws the GUI.
    /// </summary>
    private void DrawGUI()
    {
        try
        {
            if (vessel != FlightGlobals.ActiveVessel)
                return;
        }
        catch (Exception)
        {
            return;
        }

        if (RendezMe.claimingController != this)
            return;

        GUI.skin = HighLogic.Skin;
        WindowPos = GUILayout.Window(WindowIDBase, WindowPos, WindowGUI, "RendezMe", GUILayout.MinWidth(200));
    }

    #endregion

    #region Control Logic

    /// <summary>
    /// Checks  if the selected vessel is still where we expect it.
    /// </summary>
    /// <returns>
    /// The vessel.
    /// </returns>
    private bool CheckVessel()
    {
        try
        {
            // Is the ship still active?
            if (this.vessel != FlightGlobals.ActiveVessel)
                return false;
        
            // Does Vessels[selVessel] contain a vessel?
            if (FlightGlobals.Vessels.Count - 1 < _selectedVesselIndex)
                return false;
        
            // Does the ID match the vessel selected?
            int id = FlightGlobals.Vessels[_selectedVesselIndex].GetInstanceID();
            if(id == _selectedVesselInstanceId)
                return true;

            // Doesn't match, search vessels for matching id
            for (int i = 0; i < FlightGlobals.Vessels.Count; i++)
            {
                id = FlightGlobals.Vessels[i].GetInstanceID();
                if (id != _selectedVesselInstanceId) 
                        continue;

                // Found it!
                _selectedVesselIndex = i;
                return true;
            }

            // Couldn't find it.
            return false;
        }
        catch (Exception)
        {
            return false;
        }

    }

    /// <summary>
    /// Updates the vectors.
    /// </summary>
    private void UpdateVectors()
    {
        try
        {
            Vector3 up = (vessel.findWorldCenterOfMass() - vessel.mainBody.position).normalized;
            Vector3 prograde = vessel.orbit.GetRelativeVel().normalized;

            Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

            _relativeVelocity = selectedVessel.orbit.GetVel() - vessel.orbit.GetVel();
            _vectorToTarget = selectedVessel.transform.position - vessel.transform.position;
            _targetDistance = Vector3.Distance(selectedVessel.transform.position, vessel.transform.position);
                
            _relativeInclination = (float)selectedVessel.orbit.inclination - (float)vessel.orbit.inclination;

            switch (PointAt)
            {
                case Orient.RelativeVelocity:
                    _tgtFwd = -_relativeVelocity;
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
                case Orient.RelativeVelocityAway:
                    _tgtFwd = _relativeVelocity;
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
                case Orient.Target:
                    _tgtFwd = _vectorToTarget;
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
                case Orient.TargetAway:
                    _tgtFwd = -_vectorToTarget;
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
                case Orient.Normal:
                    _tgtFwd = Vector3.Cross(prograde, up);
                    _tgtUp = up;
                    break;
                case Orient.AntiNormal:
                    _tgtFwd = -Vector3.Cross(prograde, up);
                    _tgtUp = up;
                    break;
                case Orient.MatchTarget:
                    _tgtFwd = selectedVessel.transform.up;
                    _tgtUp = -selectedVessel.transform.forward;
                    break;
                case Orient.MatchTargetAway:
                    _tgtFwd = -selectedVessel.transform.up;
                    _tgtUp = -selectedVessel.transform.forward;
                    break;
                case Orient.Prograde:
                    _tgtFwd = vessel.rigidbody.velocity.normalized;
                    _tgtUp = new Vector3(0, 0, 1);
                    break;
                case Orient.Retrograde:
                    _tgtFwd = -vessel.rigidbody.velocity.normalized;
                    _tgtUp = new Vector3(0, 0, 1);
                    break;
                case Orient.Align:
                    Vector3 vecAlign = _vectorToTarget / 100.0f;
                    _tgtFwd = _relativeVelocity + vecAlign;
                    if (_tgtFwd.magnitude >= 600.0f)
                        _tgtFwd /= (_tgtFwd.magnitude / 600.0f);
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
                case Orient.UnAlign:
                    Vector3 vecUnAlign = _vectorToTarget / 100.0f;
                    _tgtFwd = -_relativeVelocity - vecUnAlign;
                    if (_tgtFwd.magnitude >= 600.0f)
                        _tgtFwd /= (_tgtFwd.magnitude / 600.0f);
                    _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                    break;
            }

            if (m_autoRendezVel.m_rendezState >= AutoRendezVel.RendezState.TurnPhase)
            {
                // Set direction in turn phase and burn phase.
                if (m_autoRendezVel.m_recordedVelocity > vessel.orbit.vel.magnitude)
                {
                    // Turn to Prograde
                    _tgtFwd = prograde;
                    _tgtUp = new Vector3(0, 0, 1);
                }
                else if (m_autoRendezVel.m_recordedVelocity < vessel.orbit.vel.magnitude)
                {
                    // Turn to Retrograde
                    _tgtFwd = -prograde;
                    _tgtUp = new Vector3(0, 0, 1);
                }
            }
        }
        catch (Exception)
        {
            _flyByWire = false;
            m_autoRendezVel.m_rendezState = AutoRendezVel.RendezState.Invalid;
            _homeOnRelativePosition = false;
            _killRelativeVelocity = false;
            Mode = UIMode.VESSELS;
            return;
        }

    }

    private void CalculateNearestRendezvousInSeconds(out double timeToRendezvous, out double minDeltaTime)
    {
        // Build up the times for apses for the next 8 orbits for the target.
        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        // Find the next few times of apsis for the target and the ship.
        double targetApoapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 180);
        double targetPeriapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 0);

        double[] targetApses = new double[16];
        double[] shipApses = new double[16];
        for (int i = 0; i < 8; i++)
        {
            targetApses[i * 2 + 0] = selectedVessel.orbit.GetTimeToTrue(0) + i * selectedVessel.orbit.period;
            targetApses[i * 2 + 1] = selectedVessel.orbit.GetTimeToTrue(0) + i * selectedVessel.orbit.period;
            shipApses[i * 2 + 0] = vessel.orbit.GetTimeToTrue(targetApoapsisAnomaly) + i * vessel.orbit.period;
            shipApses[i * 2 + 1] = vessel.orbit.GetTimeToTrue(targetPeriapsisAnomaly) + i * vessel.orbit.period;
        }

        // Walk the lists and find the nearest times. This could be optimized
        // but it doesn't matter.

        double closestPeriDeltaT = double.MaxValue;
        int closestPeriIndex = -1;

        double closestApoDeltaT = double.MaxValue;
        int closestApoIndex = -1;

        for(int i=0; i < 8; i++)
        {
            double shipApoT = shipApses[i*2+0];
            double shipPeriT = shipApses[i*2+1];

            for(int j=0; j < 8; j++)
            {
                double targApoT = targetApses[j * 2 + 0];
                double deltaApo = Math.Abs(shipApoT - targApoT);
                if(deltaApo < closestApoDeltaT)
                {
                    closestApoDeltaT = deltaApo;
                    closestApoIndex = j * 2 + 0;
                }

                double targPeriT = targetApses[j * 2 + 1];
                double deltaPeri = Math.Abs(shipPeriT - targPeriT);
                if (deltaPeri < closestPeriDeltaT)
                {
                    closestPeriDeltaT = deltaPeri;
                    closestPeriIndex = j * 2 + 1;
                }
            }
        }

        if(closestApoDeltaT < closestPeriDeltaT)
        {
            timeToRendezvous = shipApses[closestApoIndex];
            minDeltaTime = closestApoDeltaT;            
        }
        else
        {
            timeToRendezvous = shipApses[closestPeriIndex];
            minDeltaTime = closestPeriDeltaT;
        }
    }

    private double CalculateTimeTillNextTargetApsis()
    {
        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        double targetApoapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 180);
        double targetPeriapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 0);

        double shipTimeToTargetApoapsis = vessel.orbit.GetTimeToTrue(targetApoapsisAnomaly);
        double shipTimeToTargetPeriapsis = vessel.orbit.GetTimeToTrue(targetPeriapsisAnomaly);

        return Math.Min(shipTimeToTargetApoapsis, shipTimeToTargetPeriapsis);
    }

    private double CalculateTimeTillFurtherTargetApsis()
    {
        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        double targetApoapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 180);
        double targetPeriapsisAnomaly = selectedVessel.orbit.TranslateAnomaly(vessel.orbit, 0);

        double shipTimeToTargetApoapsis = vessel.orbit.GetTimeToTrue(targetApoapsisAnomaly);
        double shipTimeToTargetPeriapsis = vessel.orbit.GetTimeToTrue(targetPeriapsisAnomaly);

        return Math.Max(shipTimeToTargetApoapsis, shipTimeToTargetPeriapsis);
    }

    private void DriveShip(FlightCtrlState controls)
    {
        if(!CheckVessel())
            return;

        if (RendezMe.claimingController != this)
            return;

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        if(_autoAlign)
        {
            // Is it time to burn? Find soonest node.
            double timeToBurnAN = vessel.orbit.GetTimeToRelAN(selectedVessel.orbit);    // Actually displayed as DN
            double timeToBurnDN = vessel.orbit.GetTimeToRelDN(selectedVessel.orbit);    // Actually displayed as AN
            
            bool ascendingSoonest = timeToBurnAN < timeToBurnDN;
            double timeToBurnNode = ascendingSoonest ? timeToBurnAN : timeToBurnDN;

            // Figure out which way we want to burn to adjust our inclination.
            _flyByWire = true;
            if(!_autoAlignBurnTriggered)
            {
                if (_relativeInclination < 0.0)
                    PointAt = ascendingSoonest ? Orient.Normal : Orient.AntiNormal;
                else
                    PointAt = ascendingSoonest ? Orient.AntiNormal : Orient.Normal;
                _modeChanged = true;
            }

            // Auto exiting time-warp when needed.
            if (timeToBurnNode < Mathf.Abs(_relativeInclination * 2.5f) + 30.0f)
            {
                if (TimeWarp.CurrentRateIndex > 0)
                    TimeWarp.SetRate(0, false);
            }
            else if (timeToBurnNode < Mathf.Abs(_relativeInclination * 2.5f) + 120.0f)
            {
                if (TimeWarp.CurrentRateIndex > 1)
                    TimeWarp.SetRate(1, false);
            }

            // Do a burn just ahead of the ascending node - in the 5 seconds preceding.
            // FIXME_HF: We should adjust the preceding time based on relative inclination.
            // For a 1.0 degree rel inc, we only need to precedes for 10 seconds.
            float burnTimeThreshold = Mathf.Max(Mathf.Abs(_relativeInclination * 10.0f), 5.0f);
            if ((timeToBurnNode < burnTimeThreshold || _autoAlignBurnTriggered) && _headingError.magnitude < 5.0 && Math.Abs(_relativeInclination) > 0.01)
            {
                _autoAlignBurnTriggered = true;
                if (Math.Abs(_relativeInclination) > 0.08)
                {
                    controls.mainThrottle = 1.0f;                    
                }
                else if (Math.Abs(_relativeInclination) > 0.04)
                {
                    controls.mainThrottle = 0.5f;
                }
                else
                {
                    controls.mainThrottle = 0.25f;
                }

                // We also need to check if the node is getting too far away from us or not.
                if (timeToBurnNode > burnTimeThreshold * 1.5f)
                {
                    // We should check if it's not done, we have to keep working on that.
                    if (Math.Abs(_relativeInclination) > 0.02)
                    {
                        controls.mainThrottle = 0.0f;
                        _autoAlignBurnTriggered = false;
                    }
                }
            }
            else
            {
                controls.mainThrottle = 0.0f;
            }

            if (Math.Abs(_relativeInclination) < 0.02)
            {
                _autoAlignBurnTriggered = false;
                _autoAlign = false;
                controls.mainThrottle = 0.0f;
                _flyByWire = false;
                _modeChanged = true;
            }
        }

        if (_autoPhaser)
        {
            switch(_autoPhaserState)
            {
                case AutoPhaserState.Step1WaitForTargetApsis:
                    double timeLeft = CalculateTimeTillNextTargetApsis();

                    // Set the PointAt based on who is faster at that point in time.
                    _flyByWire = true;
                    if (vessel.orbit.getOrbitalSpeedAt(timeLeft) > selectedVessel.orbit.getOrbitalSpeedAt(timeLeft))
                        PointAt = Orient.Retrograde;
                    else
                        PointAt = Orient.Prograde;

                    // Advance if it's time.
                    if (timeLeft < 5.0)
                    {
                        _autoPhaserState = AutoPhaserState.Step2BurnToMatchNextApsis;
                        _autoPhaserVelocityGoal = selectedVessel.orbit.getOrbitalSpeedAt(CalculateTimeTillFurtherTargetApsis());
                        _autoPhaseBurnComplete = false;
                    }
                    break;

                case AutoPhaserState.Step2BurnToMatchNextApsis:
                    double predictedVelocity = vessel.orbit.getOrbitalSpeedAt(CalculateTimeTillFurtherTargetApsis());
                    if (_headingError.magnitude < 5.0 && !_autoPhaseBurnComplete)
                    {
                        controls.mainThrottle = 1;
                    }
                    else
                    {
                        controls.mainThrottle = 0;
                    }

                    // Advance to next state if we hit our goal.
                    if (Math.Abs(predictedVelocity - _autoPhaserVelocityGoal) < 10)
                    {
                        _autoPhaseBurnComplete = true;
                        controls.mainThrottle = 0;

                    }

                    // Wait till we pass the apsis so we don't double advance.
                    if (_autoPhaseBurnComplete && CalculateTimeTillNextTargetApsis() > 10.0)
                        _autoPhaserState = AutoPhaserState.Step3WaitForTargetApsis;
                    break;

                case AutoPhaserState.Step3WaitForTargetApsis:
                    timeLeft = CalculateTimeTillNextTargetApsis();

                    // Set the PointAt based on who is faster at that point in time.
                    _flyByWire = true;
                    PointAt = Orient.Prograde;

                    // Advance if it's time.
                    if (timeLeft < 5.0)
                    {
                        _autoPhaserState = AutoPhaserState.Step4BurnToRendezvous;
                    }

                    break;

                case AutoPhaserState.Step4BurnToRendezvous:

                    // TODO: Make sure we are only considering the apsis that
                    // is spatially similar to ours, otherwise we get in sync
                    // orbitally but go into step 5 super far away.
                    double timeToRendezvous = 0.0, minDeltaT = 0.0;
                    CalculateNearestRendezvousInSeconds(out timeToRendezvous, out minDeltaT);

                    if (minDeltaT > 5)
                        controls.mainThrottle = 0.25f;
                    else
                    {
                        controls.mainThrottle = 0.0f;
                        _autoPhaserState = AutoPhaserState.Step5WaitForRendezvous;
                    }
                    break;

                case AutoPhaserState.Step5WaitForRendezvous:
                    timeToRendezvous = 0.0;
                    minDeltaT = 0.0;
                    CalculateNearestRendezvousInSeconds(out timeToRendezvous, out minDeltaT);

                    if(timeToRendezvous < 2)
                        _autoPhaserState = AutoPhaserState.Step6BurnToMatchVelocity;

                    break;

                case AutoPhaserState.Step6BurnToMatchVelocity:
                    if(_relativeVelocity.magnitude > 5)
                    {
                        _flyByWire = true;
                        PointAt = Orient.RelativeVelocityAway;

                        if(_headingError.magnitude < 5)
                        {
                            if (_relativeVelocity.magnitude > 15)
                                controls.mainThrottle = 1.0f;
                            else
                                controls.mainThrottle = 0.2f;
                        }

                    }
                    else
                    {
                        // All done!
                        controls.mainThrottle = 0.0f;
                        _autoPhaser = false;
                    }
                    break;

            }
        }

        float ownRcsX = 0, ownRcsY = 0, ownRcsZ = 0;

        if(_killRelativeVelocity)
        {
            Vector3 _projectedLocalRelativeVelocity;
            float _closureRate = Vector3.Dot(_localRelativePosition, _localRelativeVelocity) / _localRelativePosition.magnitude;
            Vector3 _projectedClosureVector = new Vector3(_localRelativePosition.x, _localRelativePosition.y, _localRelativePosition.z);
            _projectedClosureVector.Normalize();
            _projectedClosureVector *= _closureRate;
            _projectedLocalRelativeVelocity = _localRelativeVelocity - _projectedClosureVector;
            float _criticalRatio = _projectedLocalRelativeVelocity.magnitude / (_localRelativePosition.magnitude / -_closureRate);
            if (_criticalRatio < 0.0f) _criticalRatio = 0.1f;

            tick++;
            bool _adjustInThisRound = false;
            float kP = 2.0f;
            float dutyRatio = 1.0f;
            if (_localRelativePosition.magnitude > 400000)
            {
                kP = 3.5f;
                dutyRatio = _criticalRatio * 0.3f;
                dutyRatio = Math.Min(dutyRatio, 0.8f);
                if (tick <= dutyRatio * 100.0f)
                {
                    _adjustInThisRound = true;
                }
                if (tick > 100)
                {
                    tick = 0;
                }
            }
            else if (_localRelativePosition.magnitude > 200000)
            {
                kP = 3.0f;
                dutyRatio = _criticalRatio * 0.6f;
                dutyRatio = Math.Min(dutyRatio, 0.9f);
                if (tick <= dutyRatio * 60.0f)
                {
                    _adjustInThisRound = true;
                }
                if (tick > 60)
                {
                    tick = 0;
                }
            }
            else if (_localRelativePosition.magnitude > 100000)
            {
                kP = 1.5f;
                dutyRatio = _criticalRatio * 1.5f;
                dutyRatio = Math.Min(dutyRatio, 1.0f);
                if (tick <= dutyRatio * 40.0f)
                {
                    _adjustInThisRound = true;
                }
                if (tick > 40)
                {
                    tick = 0;
                }
            }
            else if (_localRelativePosition.magnitude > 50000)
            {
                kP = 2.0f;
                dutyRatio = 0.5f;
                if (tick <= dutyRatio * 10.0f)
                {
                    _adjustInThisRound = true;
                }
                if (tick > 10)
                {
                    tick = 0;
                }
            }
            else
            {
                kP = 1.8f;
                dutyRatio = 0.5f;
                if (tick <= dutyRatio * 4.0f)
                {
                    _adjustInThisRound = true;
                }
                if (tick > 4)
                {
                    tick = 0;
                }
            }

            if (_adjustInThisRound)
            {
                ownRcsX = -_projectedLocalRelativeVelocity.x * kP;
                ownRcsY = -_projectedLocalRelativeVelocity.z * kP;
                ownRcsZ = -_projectedLocalRelativeVelocity.y * kP;
                    
                //controls.X = Mathf.Clamp(-_projectedLocalRelativeVelocity.x * kP, -1.0f, 1.0f);
                //controls.Y = Mathf.Clamp(-_projectedLocalRelativeVelocity.z * kP, -1.0f, 1.0f);
                //controls.Z = Mathf.Clamp(-_projectedLocalRelativeVelocity.y * kP, -1.0f, 1.0f);
            }

            // Real old codes...
            //controls.X = Mathf.Clamp(-_localRelativeVelocity.x * 8.0f, -1.0f, 1.0f);
            //controls.Y = Mathf.Clamp(-_localRelativeVelocity.z * 8.0f, -1.0f, 1.0f);
            //controls.Z = Mathf.Clamp(-_localRelativeVelocity.y * 8.0f, -1.0f, 1.0f);

            //if (_localRelativeVelocity.magnitude < 0.1)
                //_killRelativeVelocity = false;
        } 
        else if (_homeOnRelativePosition)
        {
            Vector3 targetGoalPos = _park_offset;
            targetGoalPos = selectedVessel.transform.localToWorldMatrix.MultiplyPoint(targetGoalPos);
            targetGoalPos = vessel.transform.worldToLocalMatrix.MultiplyPoint(targetGoalPos);

            Vector3 relPos = targetGoalPos;

            float kP = 0.01f;
            float kD = 0.2f;

            float cX, cY, cZ;
            cX = Mathf.Clamp(-relPos.x * kP, -2.0f, 2.0f) - _localRelativeVelocity.x * kD;
            cY = Mathf.Clamp(-relPos.z * kP, -2.0f, 2.0f) - _localRelativeVelocity.z * kD;
            cZ = Mathf.Clamp(-relPos.y * kP, -2.0f, 2.0f) - _localRelativeVelocity.y * kD;

            // Currently we adjust to 3x power for v0.18.
            ownRcsX += Mathf.Clamp(cX * 4.0f, -0.75f, 0.75f);
            ownRcsY += Mathf.Clamp(cY * 4.0f, -0.75f, 0.75f);
            ownRcsZ += Mathf.Clamp(cZ * 4.0f, -0.75f, 0.75f);

            //controls.X = Mathf.Clamp(controls.X + cX * 4.0f, -1.0f, 1.0f);
            //controls.Y = Mathf.Clamp(controls.Y + cY * 4.0f, -1.0f, 1.0f);
            //controls.Z = Mathf.Clamp(controls.Z + cZ * 4.0f, -1.0f, 1.0f);
        }

        if (m_autoRendezVel.m_rendezState < AutoRendezVel.RendezState.TurnPhase)
        {
            m_autoRendezVel.driveShip(controls);
        }

        if (_flyByWire)
        {
            if ((m_autoRendezVel.m_rendezState == AutoRendezVel.RendezState.Invalid) || (m_autoRendezVel.m_rendezState >= AutoRendezVel.RendezState.TurnPhase))
            {

                Quaternion offset = Quaternion.identity;
                if (_selectedFlyMode >= 4 && _selectedFlyMode <= 5)
                {
                    offset = Quaternion.Euler(new Vector3(-_tgt_act_pit, _tgt_act_yaw, -_tgt_act_rol));
                }

                Quaternion tgt = Quaternion.LookRotation(_tgtFwd, _tgtUp);
                tgt = tgt * offset;
                Quaternion delta =
                    Quaternion.Inverse(Quaternion.Euler(90, 0, 0) * Quaternion.Inverse(vessel.transform.rotation) * tgt);

                Vector3 forwardVec = Vector3.forward;
                Matrix4x4 trs = Matrix4x4.TRS(Vector3.zero, tgt, Vector3.one);
                forwardVec = trs.MultiplyPoint(forwardVec);
                Vector3 tgtLocalUp = vessel.transform.worldToLocalMatrix.MultiplyPoint(vessel.transform.position + forwardVec);
                Vector3 curLocalUp = Vector3.up;

                float turnAngle = 0.0f;
                turnAngle = Mathf.Abs(Vector3.Angle(curLocalUp, tgtLocalUp));
                Vector2 rotDirection = new Vector2(tgtLocalUp.x, tgtLocalUp.z);
                rotDirection = rotDirection.normalized * turnAngle / 180.0f;

                _headingError =
                    new Vector3(rotDirection.y,
                                rotDirection.x,
                                ((delta.eulerAngles.z > 180) ? (delta.eulerAngles.z - 360.0F) : delta.eulerAngles.z) / 180.0F);

                float scale = 1.0f;
                if (Mathf.Abs(_headingError.x) >= 0.05 || Mathf.Abs(_headingError.y) >= 0.05)
                {
                    _headingError.z = 0.0f;
                    float maxComponent = Mathf.Abs(Mathf.Max(_headingError.x, _headingError.y));
                    if (maxComponent >= 0.5f)
                        scale = 0.5f / maxComponent;
                }

                Vector3 curVectorLocal = Vector3.up;
                Vector3 prevVectorLocal = vessel.transform.worldToLocalMatrix.MultiplyPoint(vessel.transform.position + _prevVector);
                float turnRate = Vector3.Angle(curVectorLocal, prevVectorLocal);
                Vector2 prevTurn = new Vector2(prevVectorLocal.z, prevVectorLocal.x);
                prevTurn.Normalize();
                prevTurn *= turnRate / 180.0f;

                if (m_autoRendezVel.m_rendezState == AutoRendezVel.RendezState.TurnPhase)
                {
                    m_autoRendezVel.setHeadingError(new Vector2(_headingError.x, _headingError.y).magnitude * 180.0);
                    m_autoRendezVel.setTurnRate(turnRate / TimeWarp.fixedDeltaTime);
                }

                _integral += _headingError * scale * TimeWarp.fixedDeltaTime;
                _deriv = new Vector3(prevTurn.x, prevTurn.y, _headingError.z - _prevError.z) / TimeWarp.fixedDeltaTime;
                _act = Kp * _headingError * scale + Ki * _integral + Kd * _deriv;
                _prevError = _headingError;
                _prevVector = vessel.transform.localToWorldMatrix.MultiplyPoint(Vector3.up) - vessel.transform.position;

                float ownCtrlPitch = -Mathf.Clamp(_act.x, -1.5f, 1.5f);
                float ownCtrlYaw = Mathf.Clamp(_act.y, -1.5f, 1.5f);
                float ownCtrlRoll = Mathf.Clamp(_act.z, -1.5f, 1.5f);

                m_autoRendezVel.driveShip(controls);

                Vector3 axisInLocalSpace = new Vector3(-ownCtrlPitch, -ownCtrlRoll, -ownCtrlYaw);
                Vector3 axisInWorldSpace = vessel.transform.TransformDirection(axisInLocalSpace);
                Vector3 axisInRefSpace = vessel.ReferenceTransform.InverseTransformDirection(axisInWorldSpace);
                
                controls.pitch = Mathf.Clamp(controls.pitch - axisInRefSpace.x, -1.0F, 1.0F);
                controls.yaw = Mathf.Clamp(controls.yaw - axisInRefSpace.z, -1.0F, 1.0F);
                controls.roll = Mathf.Clamp(controls.roll - axisInRefSpace.y, -1.0F, 1.0F);
            }
        }

        Vector3 rcsVecInLocalSpace = new Vector3(-ownRcsX, -ownRcsZ, -ownRcsY);
        Vector3 rcsVecInWorldSpace = vessel.transform.TransformDirection(rcsVecInLocalSpace);
        Vector3 rcsVecInRefSpace = vessel.ReferenceTransform.InverseTransformDirection(rcsVecInWorldSpace);
        
        controls.X = Mathf.Clamp(controls.X - rcsVecInRefSpace.x, -1.0F, 1.0F);
        controls.Y = Mathf.Clamp(controls.Y - rcsVecInRefSpace.z, -1.0F, 1.0F);
        controls.Z = Mathf.Clamp(controls.Z - rcsVecInRefSpace.y, -1.0F, 1.0F);

    }

    #endregion

    #region Kerbel Interface

    protected override void onFlightStart()
    {
    	try
    	{
    	    RenderingManager.AddToPostDrawQueue(3, DrawGUI);
    	    addedToDrawQueue = true;
        }
        catch (Exception)
        {
        }
        
        try
        {
            RendezMe.claimControl(this.vessel, this);
            if (RendezMe.claimingController == this)
            {
                //Debug.Log("Claim succeeded");
            }
            else
            {
               //Debug.Log("Claim failed");
            }
        }
        catch (Exception)
        {
        }

        obj.layer = 9;
        mapCamera = (MapView)GameObject.FindObjectOfType(typeof(MapView));

        line = obj.AddComponent<LineRenderer>();
        parkLineX = parkLineXObj.AddComponent<LineRenderer>();
        parkLineY = parkLineYObj.AddComponent<LineRenderer>();
        parkLineZ = parkLineZObj.AddComponent<LineRenderer>();
        relVelLine = relVelLineObj.AddComponent<LineRenderer>();
        autoPilotDirectionLine = autoPilotDirectionLineObj.AddComponent<LineRenderer>();

        line.transform.parent = null;
        line.useWorldSpace = false;
        line.material = new Material(Shader.Find("Particles/Additive"));
        line.SetColors(Color.yellow, Color.red);

        parkLineX.transform.parent = vessel.transform;
        parkLineX.useWorldSpace = false;
        parkLineX.material = new Material(Shader.Find("Particles/Additive"));
        parkLineX.SetColors(Color.red, Color.red);

        parkLineY.transform.parent = vessel.transform;
        parkLineY.useWorldSpace = false;
        parkLineY.material = new Material(Shader.Find("Particles/Additive"));
        parkLineY.SetColors(Color.green, Color.green);

        parkLineZ.transform.parent = vessel.transform;
        parkLineZ.useWorldSpace = false;
        parkLineZ.material = new Material(Shader.Find("Particles/Additive"));
        parkLineZ.SetColors(Color.blue, Color.blue);

        relVelLine.transform.parent = vessel.transform;
        relVelLine.useWorldSpace = false;
        relVelLine.material = new Material(Shader.Find("Particles/Additive"));
        relVelLine.SetColors(Color.yellow, Color.yellow);

        autoPilotDirectionLine.transform.parent = vessel.transform;
        autoPilotDirectionLine.useWorldSpace = false;
        autoPilotDirectionLine.material = new Material(Shader.Find("Particles/Additive"));
        autoPilotDirectionLine.SetColors(Color.gray, Color.gray);
    }

    public void drawApproach(int lines, bool enabled)
    {
        if (!enabled)
        {   
            line.enabled = false;
            relVelLine.enabled = false;
            autoPilotDirectionLine.enabled = false;
            return;
        }

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;
        if (selectedVessel != null)
        {
            if (MapView.MapIsEnabled)
            {
                try
                {
                    relVelLine.enabled = false;
                    autoPilotDirectionLine.enabled = false;

                    Orbit to_body = selectedVessel.orbit;
                    double dT = (1.0 * FlightGlobals.ActiveVessel.orbit.period) / lines;

                    line.SetVertexCount(lines * 3);
                    line.enabled = true;
                    line.useWorldSpace = true;
					line.SetWidth((0.002f * MapView.MapCamera.Distance), (0.002f * MapView.MapCamera.Distance));
                    double time = Planetarium.GetUniversalTime();
                    for (int j = 0; j < lines; j++)
                    {
                        line.SetPosition(j * 3, ScaledSpace.LocalToScaledSpace(FlightGlobals.ActiveVessel.orbit.getPositionAtUT(time)));
                        line.SetPosition(j * 3 + 1, ScaledSpace.LocalToScaledSpace(to_body.getPositionAtUT(time)));
                        time = time + dT;
                        line.SetPosition(j * 3 + 2, ScaledSpace.LocalToScaledSpace(to_body.getPositionAtUT(time)));
                    }
                }
                catch (Exception)
                { 
                }
            }
            else
            {
                line.enabled = false;

                // Calculate the LOS rate here.
                Vector3 relVelVec = -_relativeVelocity;
                Vector3 normalizedRelVel = relVelVec.normalized;
                float relVelScale = 1.0f;
                if (relVelVec.magnitude >= 10.0f)
                {
                    relVelScale = 51.0f - 1 / (relVelVec.magnitude * 5.0f - 9.0f);
                }
                else
                {
                    relVelScale = relVelVec.magnitude * 5.0f;
                }
                
                relVelLine.SetVertexCount(2);

                relVelLine.enabled = true;
                relVelLine.useWorldSpace = false;
                relVelLine.transform.parent = vessel.transform;
                relVelLine.transform.localPosition = Vector3.zero;
                relVelLine.transform.localEulerAngles = Vector3.zero;
                relVelLine.transform.localScale = Vector3.one;
                relVelLine.SetWidth(0.5f, 0.0f);
                relVelLine.SetPosition(0, Vector3.zero);
                relVelLine.SetPosition(1, vessel.transform.worldToLocalMatrix.MultiplyPoint(vessel.transform.position + normalizedRelVel * relVelScale));

                if (_flyByWire == true)
                {
                    Vector3 centerPoint = vessel.findLocalCenterOfMass();
                    Vector3 forwardVec = _tgtFwd;
                    if (_selectedFlyMode >= 4 && _selectedFlyMode <= 5)
                    {
                        forwardVec = Vector3.forward;
                        Quaternion tgt = Quaternion.LookRotation(_tgtFwd, _tgtUp);
                        tgt = tgt * Quaternion.Euler(new Vector3(-_tgt_act_pit, _tgt_act_yaw, -_tgt_act_rol));
                        Matrix4x4 trs = Matrix4x4.TRS(Vector3.zero, tgt, Vector3.one);
                        forwardVec = trs.MultiplyPoint(forwardVec);
                    }
                    forwardVec *= 30.0f / forwardVec.magnitude;
                    
                    autoPilotDirectionLine.SetVertexCount(2);

                    autoPilotDirectionLine.enabled = true;
                    autoPilotDirectionLine.useWorldSpace = false;
                    autoPilotDirectionLine.transform.parent = vessel.transform;
                    autoPilotDirectionLine.transform.localPosition = Vector3.zero;
                    autoPilotDirectionLine.transform.localEulerAngles = Vector3.zero;
                    autoPilotDirectionLine.transform.localScale = Vector3.one;
                    autoPilotDirectionLine.SetWidth(0.1f, 0.1f);
                    autoPilotDirectionLine.SetPosition(0, centerPoint);
                    autoPilotDirectionLine.SetPosition(1, centerPoint + vessel.transform.worldToLocalMatrix.MultiplyPoint(vessel.transform.position + forwardVec));
                }
                else
                {
                    autoPilotDirectionLine.enabled = false;
                }
            }
        }
        else
        {
            line.enabled = false;
            relVelLine.enabled = false;
            autoPilotDirectionLine.enabled = false;
        }

    }

    public void drawParkOffset(bool enabled)
    {
        if (!enabled)
        {   
            parkLineX.enabled = false;
            parkLineY.enabled = false;
            parkLineZ.enabled = false;
            return;
        }

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;
        if (selectedVessel != null)
        {
            if (!MapView.MapIsEnabled)
            {
                Vector3 targetGoalPos = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.localToWorldMatrix.MultiplyPoint(_park_offset));
                Vector3 targetUp = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.localToWorldMatrix.MultiplyPoint(_park_offset + Vector3.up)) - targetGoalPos;
                Vector3 targetRight = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.localToWorldMatrix.MultiplyPoint(_park_offset + Vector3.right)) - targetGoalPos;
                Vector3 targetForward = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.localToWorldMatrix.MultiplyPoint(_park_offset + Vector3.forward)) - targetGoalPos;

                parkLineX.SetVertexCount(2);
                parkLineX.enabled = true;
                parkLineX.useWorldSpace = false;
                parkLineX.transform.parent = vessel.transform;
                parkLineX.transform.localPosition = Vector3.zero;
                parkLineX.transform.localEulerAngles = Vector3.zero;
                parkLineX.transform.localScale = Vector3.one;
                parkLineX.SetWidth(0.2f, 0.2f);
                parkLineX.SetPosition(0, targetGoalPos - targetRight * 50.0f);
                parkLineX.SetPosition(1, targetGoalPos + targetRight * 50.0f);
                
                parkLineY.SetVertexCount(2);
                parkLineY.enabled = true;
                parkLineY.useWorldSpace = false;
                parkLineY.transform.parent = vessel.transform;
                parkLineY.transform.localPosition = Vector3.zero;
                parkLineY.transform.localEulerAngles = Vector3.zero;
                parkLineY.transform.localScale = Vector3.one;
                parkLineY.SetWidth(0.2f, 0.2f);
                parkLineY.SetPosition(0, targetGoalPos - targetUp * 50.0f);
                parkLineY.SetPosition(1, targetGoalPos + targetUp * 50.0f);
                
                parkLineZ.SetVertexCount(2);
                parkLineZ.enabled = true;
                parkLineZ.useWorldSpace = false;
                parkLineZ.transform.parent = vessel.transform;
                parkLineZ.transform.localPosition = Vector3.zero;
                parkLineZ.transform.localEulerAngles = Vector3.zero;
                parkLineZ.transform.localScale = Vector3.one;
                parkLineZ.SetWidth(0.2f, 0.2f);
                parkLineZ.SetPosition(0, targetGoalPos - targetForward * 50.0f);
                parkLineZ.SetPosition(1, targetGoalPos + targetForward * 50.0f);
                
            }
            else
            {
                parkLineX.enabled = false;
                parkLineY.enabled = false;
                parkLineZ.enabled = false;
            }
        }
        else
        {
            parkLineX.enabled = false;
            parkLineY.enabled = false;
            parkLineZ.enabled = false;
        }

    }

    protected override void onDisconnect()
    {
        _flyByWire = false;
        
        //Debug.Log("Separated Control");
    }

    protected override void onPartDestroy() //Called when the part is destroyed
    {
        _flyByWire = false;
        if(addedToDrawQueue)
        RenderingManager.RemoveFromPostDrawQueue(3, DrawGUI);
    }

    protected override void onPartUpdate()
    {
        _rendezvousRecalculationTimer += TimeWarp.deltaTime;

        // Trying to claim control in each update.
        claimControl(this.vessel, this);
        
        if (Mode == UIMode.SYNC)
        {
            if (CheckVessel() == true && RendezMe.claimingController == this)
            {
                PerformSyncPartLogic();
            }
            else
            {
                drawApproach(0, false);
                drawParkOffset(false);
            }
        }

        if (Mode == UIMode.RENDEZVOUS)
        {
            if (CheckVessel() == true && RendezMe.claimingController == this)
            {
                drawApproach(40, true);
                drawParkOffset(_homeOnRelativePosition);
            }
            else
            {
                drawApproach(0, false);
                drawParkOffset(false);
            }
        }
        else
        {
            drawApproach(0, false);
            drawParkOffset(false);
        }

        m_autoRendezVel.setShips(this.vessel, FlightGlobals.Vessels[_selectedVesselIndex]);
        m_autoRendezVel.update(ref _flyByWire, ref _modeChanged);

        UpdateVectors();

        if (_modeChanged)
        {
            WindowPos.width = WindowPos.height = 20;
            _modeChanged = false;
        }
    }

    private void PerformSyncPartLogic()
    {
        // What anomaly are we trying to rendezvous at?
        switch (SyncMode)
        {
            case SynchronizationType.ShipApoapsis:
                _rendezvousAnomaly = 180;
                break;
            case SynchronizationType.ShipPeriapsis:
                _rendezvousAnomaly = 0;
                break;
            case SynchronizationType.TargetApoapsis:
                _rendezvousAnomaly = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 180);
                break;
            case SynchronizationType.TargetPeriapsis:
                _rendezvousAnomaly = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 0);
                break;
        }

        // Only recalculate if enough time has elapsed.
        if (_rendezvousRecalculationTimer < .1) 
            return;

        // Find the time away from the anomaly we'll be at rendezvous.
        for (int i = 0; i < 8; i++)
        {
            _shipTimeToRendezvous[i] = (float) vessel.orbit.GetTimeToTrue(_rendezvousAnomaly) + (float) vessel.orbit.period*i;
            _targetTimeToRendezvous[i] = (float) vessel.orbit.Syncorbits(FlightGlobals.Vessels[_selectedVesselIndex].orbit, _rendezvousAnomaly, i);
        }

        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                if (i == 0 && j == 0)
                {
                    _minimumPredictedTimeFromTarget = Math.Abs(_shipTimeToRendezvous[i] - _targetTimeToRendezvous[j]);
                    _closestApproachOrbit = _closestTargetOrbit = 0;
                }

                if (_minimumPredictedTimeFromTarget > Math.Abs(_shipTimeToRendezvous[i] - _targetTimeToRendezvous[j]))
                {
                    _closestApproachOrbit = i;
                    _closestTargetOrbit = j;
                    _minimumPredictedTimeFromTarget = Math.Abs(_shipTimeToRendezvous[i] - _targetTimeToRendezvous[j]);
                }
            }
        }

        //double junk;
        //CalculateNearestRendezvousInSeconds(out junk, out _minimumPredictedTimeFromTarget);

        // Update the display.
        for (int i = 0; i < 8; i++)
        {
            _syncString[i] = i.ToString() + "			" + _shipTimeToRendezvous[i].ToString("f0") + "			" + _targetTimeToRendezvous[i].ToString("f0");
        }

        // Reset the timer.
        _rendezvousRecalculationTimer = 0;
    }

    protected override void onPartStart()
    {
        vessel.OnFlyByWire += new FlightInputCallback(DriveShip);
        if ((WindowPos.x == 0) && (WindowPos.y == 0))
        {
            WindowPos = new Rect(Screen.width - 360, 10, 10, 10);
        }
    }

    #endregion
}