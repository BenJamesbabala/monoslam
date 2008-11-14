/* 
    MonoSLAM:  A vision based SLAM program
    Based upon SceneLib, by Andrew Davison ( http://www.doc.ic.ac.uk/~ajd )
    Copyright (C) 2006  Bob Mottram

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
*/

using System;
using System.Diagnostics;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using sentience;

namespace SceneLibrary
{
    /// <summary>
    /// this class is used to load and save settings
    /// </summary>
    public class Settings
    {
        //public Settings(StreamReader stream) { load(stream); }

        private ArrayList m_sections = new ArrayList();
        //private String m_empty = "";

        /// <summary>
        /// A section from the configuration stream. Sections are denoted by square
        /// brackets, and consist of a number of Entries, each of which is a string
        /// starting on a new line and followed by '='.
        /// </summary>
        public class Section
        {
            public String Label() { return m_label; }
        
            private String m_label;                        //the title for this section
            private ArrayList m_entries = new ArrayList(); //entries with a name followed by equals sign
            private ArrayList m_values = new ArrayList();  //contains list of values associated with each entry

            public Section(String label)
            {
                m_label = label;
            }

            /// <summary>
            /// Write the section to a stream.
            /// </summary>
            /// <param name="binfile"></param>
            public void Write(StreamWriter stream)
            {
                String s, v;
                int no_of_entries, no_of_values;

                stream.WriteLine("[" + Label() + "]");
                no_of_entries = m_entries.Count;
                //stream.WriteLine(no_of_entries);
                for (int i = 0; i < m_entries.Count; i++)
                {
                    s = (String)m_entries[i];
                    stream.Write(s + "=");
                    no_of_values = ((ArrayList)m_values[i]).Count;
                    //stream.WriteLine(no_of_values);
                    for (int j = 0; j < no_of_values; j++)
                    {
                        v = (String)(((ArrayList)m_values[i])[j]);
                        stream.WriteLine(v);
                    }
                }
            }

            /// <summary>
            /// read a section from the given stream
            /// </summary>
            /// <param name="stream"></param>
            public void Read(StreamReader stream, ref String next_line)
            {
                String s, v, line;
                int i;
                bool end_of_section, end_of_entry;

                //read the label
                if (next_line == "")
                    line = stream.ReadLine();
                else
                    line = next_line;

                if (line != null)
                {
                    if (line[0] == '[')
                    {
                        m_label = line.Substring(1, line.Length - 2);
                        i = 0;
                        end_of_section = false;
                        m_entries.Clear();
                        m_values.Clear();
                        while (!end_of_section)
                        {
                            s = stream.ReadLine();

                            if (s == null)
                                end_of_section = true;
                            else
                            {
                                if (s[0] == '[')
                                {
                                    end_of_section = true;
                                    next_line = s;
                                }
                                else
                                {
                                    if (s.Contains("="))
                                    {
                                        String[] values = s.Split('=');
                                        m_entries.Add(values[0]);
                                        m_values.Add(new ArrayList());
                                        ((ArrayList)m_values[i]).Add(values[1]);
                                        i++;
                                    }
                                    else
                                    {
                                        end_of_entry = false;
                                        while (!end_of_entry)
                                        {
                                            v = stream.ReadLine();
                                            if (v == null)
                                            {
                                                end_of_entry = true;
                                                end_of_section = true;
                                            }
                                            else
                                            {
                                                if (v[0] == '[')
                                                {
                                                    end_of_entry = true;
                                                    end_of_section = true;
                                                    next_line = v;
                                                }
                                                else
                                                {
                                                    ((ArrayList)m_values[i]).Add(v);
                                                }
                                            }

                                            if (stream.EndOfStream)
                                            {
                                                end_of_entry = true;
                                                end_of_section = true;
                                            }
                                        }
                                    }
                                }
                            }

                            if (stream.EndOfStream) end_of_section = true;

                        }
                    }
                }
            }


            /// <summary>
            /// Creates a new entry for this section and adds data to it. If the entry already
            /// exists, nothing is done.
            /// </summary>
            /// <param name="entry">The entry to create</param>
            /// <param name="data">The data to add to the new entry</param>
            /// <returns>true on success, or false if the entry already exists.</returns>
            public bool add_entry(String entry, String data)
            {
                // Is this entry already here?
                if (m_entries.Contains(entry))
                    return false;
                else
                {
                    m_entries.Add(entry);
                    ArrayList values = new ArrayList();
                    values.Add(data);
                    m_values.Add(values);
                    return true;
                }
            }


            /// <summary>
            /// Adds data to an existing entry (on a new line). If the entry doesn't exist, nothing is done.
            /// </summary>
            /// <param name="entry">The entry to add to</param>
            /// <param name="data">The data to add</param>
            /// <returns>true on success, or false if the entry</returns>
            public bool add_values_to_entry(String entry, String data)
            {
                // Is this entry already here?
                if (!m_entries.Contains(entry))
                    return false;
                else
                {
                    int idx = get_entry_index(entry);
                    if (idx > -1)
                    {
                        ((ArrayList)m_values[idx]).Add(data);
                        return true;
                    }
                    else
                        return false;
                }
            }


            /// <summary>
            /// Returns the data associated with a given entry.
            /// </summary>
            /// <param name="entry"></param>
            /// <returns></returns>
            public ArrayList get_entry(String entry)
            {
                String s;
                ArrayList found = null;

                int i = 0;
                while ((i < m_entries.Count) && (found == null))
                {
                    s = (String)m_entries[i];
                    if (s == entry) found = (ArrayList)m_values[i];
                    i++;
                }
                return (found);
            }


            /// <summary>
            /// Returns the index number of the given entry.
            /// </summary>
            /// <param name="entry"></param>
            /// <returns></returns>
            public int get_entry_index(String entry)
            {
                String s;
                int idx = -1;

                int i = 0;
                while ((i < m_entries.Count) && (i < 0))
                {
                    s = (String)m_entries[i];
                    if (s == entry) idx = i;
                    i++;
                }
                return (idx);
            }
        }

        /// <summary>
        /// create a default initialisation file
        /// </summary>
        /// <param name="initialisation_file"></param>
        public void createDefault(String initialisation_file, float target_width_mm, float target_height_mm, float target_distance_mm)
        {
            Section new_section;
            StreamWriter stream = File.CreateText(initialisation_file);

            float half_width_metres = target_width_mm / 2000;
            float half_height_metres = target_height_mm / 2000;
            float target_dist_metres = -target_distance_mm / 1000;

            m_sections.Clear();

            new_section = new Section("Models");
            new_section.add_entry("MotionModel", "IMPULSE_THREED");
            new_section.add_entry("NewFeatureMeasurementModel", "CAMERA_WIDE_LINE_INIT");
            m_sections.Add(new_section);
            new_section = new Section("CAMERA_WIDE_POINT");
            new_section.add_entry("Camera", "UNIBRAIN_WIDE_CAMERA");
            m_sections.Add(new_section);
            new_section = new Section("CAMERA_WIDE_LINE_INIT");
            new_section.add_entry("Camera", "UNIBRAIN_WIDE_CAMERA");
            m_sections.Add(new_section);
            new_section = new Section("UNIBRAIN_WIDE_CAMERA");
            new_section.add_entry("CameraParameters", "320,240,162,125,195,195,9e-06,1");
            m_sections.Add(new_section);
            new_section = new Section("IMPULSE_THREED");
            m_sections.Add(new_section);
            new_section = new Section("InitialState");
            new_section.add_entry("MotionModel", "IMPULSE_THREED");
            new_section.add_entry("xv", "0.0,0.0," + Convert.ToString(target_dist_metres) + ",1.0,0.0,0.0,0.0,0.0,0.0,-0.1,0.0,0.0,0.01");
            new_section.add_entry("Pxx", "0.0004,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0004,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0004,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0," +
                                         "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0");
            m_sections.Add(new_section);
            for (int i = 1; i <= 4; i++)
            {
                new_section = new Section("KnownFeature" + Convert.ToString(i));
                new_section.add_entry("FeatureMeasurementModel", "CAMERA_WIDE_POINT");
                new_section.add_entry("Identifier", "known_patch" + Convert.ToString(i-1) + ".bmp");
                switch (i)
                {
                    case (1):
                        {
                            new_section.add_entry("yi", Convert.ToString(half_width_metres) + "," +
                                                        Convert.ToString(half_height_metres) + ",0.0");
                            break;
                        }
                    case (2):
                        {
                            new_section.add_entry("yi", Convert.ToString(-half_width_metres) + "," +
                                                        Convert.ToString(half_height_metres) + ",0.0");
                            break;
                        }
                    case (3):
                        {
                            new_section.add_entry("yi", Convert.ToString(half_width_metres) + "," +
                                                        Convert.ToString(-half_height_metres) + ",0.0");
                            break;
                        }
                    case (4):
                        {
                            new_section.add_entry("yi", Convert.ToString(-half_width_metres) + "," +
                                                        Convert.ToString(-half_height_metres) + ",0.0");
                            break;
                        }
                }
                new_section.add_entry("xp_orig", "0.0,0.0," + Convert.ToString(target_dist_metres) + "," +
                                                 "1.0,0.0,0.0,0.0");
                m_sections.Add(new_section);
            }

            save(stream);
            stream.Close();
        }

        /// <summary>
        /// Returns the data associated with a particular section and entry
        /// </summary>
        /// <param name="section"></param>
        /// <param name="entry"></param>
        /// <returns></returns>
        public ArrayList get_entry(String section, String entry)
        {
            Section psection = get_section(section);
            if (psection == null)
                return null;
            else
                return psection.get_entry(entry);
        }


        /// <summary>
        /// load data from a stream
        /// </summary>
        /// <param name="stream"></param>
        public void load(StreamReader stream)
        {
            String next_section = "";
            m_sections.Clear();

            while (!stream.EndOfStream)
            {
                Section new_section = new Section("");
                new_section.Read(stream, ref next_section);
                if (new_section.Label() != "") m_sections.Add(new_section);
            }
        }

        /// <summary>
        /// save data to a stream
        /// </summary>
        /// <param name="stream"></param>
        public void save(StreamWriter stream)
        {
            //stream.WriteLine((int)(m_sections.Count));
            for (int i = 0; i < m_sections.Count; i++)
            {
                Section section = (Section)m_sections[i];
                section.Write(stream);
            }
        }


        /// <summary>
        /// returns the section with the given name
        /// </summary>
        /// <param name="section_name"></param>
        /// <returns></returns>
        public Section get_section(String section_name)
        {
            Section s, found=null;
            int i=0;

            while ((i < m_sections.Count) && (found == null))
            {
                s = (Section)m_sections[i];
                if (s.Label() == section_name) found = s;
                i++;
            }

            return(found);
        }

        /// <summary>
        /// Gets an entry name from the next valid line of the stream. An entry is a string
        /// starting at the beginning of the line and ending in an
        /// '=', for example 'Speed='. The rest of the line, and any subsequent
        /// lines which do not contain an '=' are the data for this entry. If the
        /// next valid line does not contain an entry name, a null string is returned, and
        /// data is filled with the line from the stream.
        /// </summary>
        /// <param name="line"></param>
        /// <param name="data">The string to fill with the entry's data (either the data following the '=', or all of the line if the line is not the start of a new entry)</param>
        /// <returns>The entry name, or an empty string if this </returns>
        private String get_entry_name(String line, ref String data)
        {
            String section="";

            // We have found a section name. How long is it?
            int equals_idx = line.IndexOf("=");
            if (equals_idx > 0)
            {
                section = line.Substring(0, equals_idx);
                data = line.Substring(equals_idx + 1);
            }

            return section;
        }


    }
}
