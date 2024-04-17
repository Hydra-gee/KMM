import xml.etree.ElementTree as ET
import socket
import os
import arrow
import pandas as pd

def allowed_gai_family():
    return socket.AF_INET

def generate_gpx_file(track,ground):
    virtualTime = 0
    output_str = ""
    output_str += "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?><gpx xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" creator=\"Graphhopper\" version=\"1.1\" xmlns:gh=\"https://graphhopper.com/public/schema/gpx/1.1\">\n<metadata><copyright author=\"OpenStreetMap contributors\"/><link href=\"http://graphhopper.com\"><text>GraphHopper GPX</text></link><time>1970-01-01T00:00:00+00:00</time></metadata>\n"
    output_str += "<rte>\n"
    for j in range(0,ground.shape[0]):
        output_str += '<rtept lat="'
        output_str += str(ground.iloc[j]['lat'])
        output_str += '" lon="'
        output_str += str(ground.iloc[j]['lon'])
        output_str += '"></rtept>\n'
    output_str += "</rte>\n"
    output_str += "<trk><name>GraphHopper</name><trkseg>\n"
    for j in range(0,track.shape[0]):
        output_str += '<trkpt lat="'
        output_str += str(track.iloc[j]['lat'])
        output_str += '" lon="'
        output_str += str(track.iloc[j]['lon'])
        if 'locatetime' in track.columns and 'speed' in track.columns and 'direction' in track.columns:
            output_str += '" locatetime="'
            output_str += str(track.iloc[j]['locatetime'])
            output_str += '" speed="'
            output_str += str(track.iloc[j]['speed'])
            output_str += '" direction="'
            output_str += str(track.iloc[j]['direction'])
        output_str += '"><time>'
        if 'locatetime' not in track.iloc[j]:
            output_str += str(virtualTime)
            virtualTime += 100000
        else:
            t = int(track.iloc[j]['locatetime'])
            output_str += str(arrow.get(t).to('local'))
        output_str += "</time></trkpt>\n"
    output_str += "</trkseg></trk></gpx>"
    return output_str

def parse_xml(file_name):
    tree = ET.parse(file_name)
    root = tree.getroot()

    namespace = '{http://www.topografix.com/GPX/1/1}'

    track = pd.DataFrame(columns=["locatetime", "lat","lon","speed","direction"])
    ground = pd.DataFrame(columns=["lat", "lon"])

    for rte in root.findall(namespace + 'rte'):
        for rtept in rte.findall(namespace + 'rtept'):
            lat = float(rtept.get('lat'))
            lon = float(rtept.get('lon'))
            ground.loc[ground.shape[0]] = [lat, lon]

    for trk in root.findall(namespace + 'trk'):
        for trkseg in trk.findall(namespace + 'trkseg'):
            for trkpt in trkseg.findall(namespace + 'trkpt'):
                lat = float(trkpt.get('lat'))
                lon = float(trkpt.get('lon'))
                if 'locatetime' in trkpt.attrib:
                    locatetime = float(trkpt.get('locatetime'))
                else:
                    locatetime = None
                if 'speed' in trkpt.attrib:
                    speed = float(trkpt.get('speed'))
                else:
                    speed = None
                if 'direction' in trkpt.attrib:
                    direction = float(trkpt.get('direction'))
                else:
                    direction = None
                track.loc[track.shape[0]] = [locatetime, lat, lon,speed,direction]
    return track,ground

def csv2_to_gpx(trk_csv_path,ground_csv_path,output_path):
    trk = pd.read_csv(trk_csv_path)
    ground = pd.read_csv(ground_csv_path)
    trk_group = trk.groupby('order_no')
    rpt_group = ground.groupby('order_no')
    trk_groups = {}
    rpt_groups = {}
    for name, group in trk_group:
        trk_groups[name] = group
    for name, group in rpt_group:
        rpt_groups[name] = group
    for key in trk_groups:
        if not key in rpt_groups:
            continue
        track = trk_groups[key]
        ground = rpt_groups[key]
        output_str = generate_gpx_file(track,ground)
        with open(output_path + "\\" + str(int(key[2:])) + ".gpx", 'w') as f:
            f.write(output_str)

    # # 步骤4: 创建字典，遍历分组，添加到字典中
    # order_no_dict = {}
    # for name, group in grouped:
    #     order_no_dict[name] = group

def csv1_to_gpx(track_path,ground_path,output_path):
    FILE_NUM = 2700
    for i in range(0, FILE_NUM):
        virtualTime = 0
        ground = pd.read_csv(ground_path + "/" + str(i) + ".csv")
        track = pd.read_csv(track_path + "/" + str(i) + ".csv")
        output_str = ""
        output_str += "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?><gpx xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" creator=\"Graphhopper\" version=\"1.1\" xmlns:gh=\"https://graphhopper.com/public/schema/gpx/1.1\">\n<metadata><copyright author=\"OpenStreetMap contributors\"/><link href=\"http://graphhopper.com\"><text>GraphHopper GPX</text></link><time>1970-01-01T00:00:00+00:00</time></metadata>\n"
        output_str += "<rte>\n"
        for j in range(0, ground.shape[0]):
            output_str += '<rtept lat="'
            output_str += str(ground.iloc[j]['lat'])
            output_str += '" lon="'
            output_str += str(ground.iloc[j]['lon'])
            output_str += '"></rtept>\n'
        output_str += "</rte>\n"
        output_str += "<trk><name>GraphHopper</name><trkseg>\n"
        for j in range(0, track.shape[0]):
            output_str += '<trkpt lat="'
            output_str += str(track.iloc[j]['lat'])
            output_str += '" lon="'
            output_str += str(track.iloc[j]['lon'])
            output_str += '" locatetime="'
            output_str += str(track.iloc[j]['locatetime'])
            output_str += '" speed="'
            output_str += str(track.iloc[j]['speed'])
            output_str += '" direction="'
            output_str += str(track.iloc[j]['direction'])
            output_str += '"><time>'
            # output_str += str(dt.datetime.fromtimestamp(int(track.iloc[j]['locatetime'])).isoformat())
            if 'locatetime' not in track.iloc[j]:
                output_str += str(virtualTime)
                virtualTime += 100000
            else:
                output_str += str(arrow.get(int(track.iloc[j]['locatetime'])).to('local'))
            output_str += "</time></trkpt>\n"
        output_str += "</trkseg></trk></gpx>"
        with open(output_path + "/" + str(i) + ".gpx", 'w') as f:
            f.write(output_str)


#convert the .gpx format to a csv format compatible with the AMM algorithm.
def gpx_to_AMM(gpx_path,output_path):
    for filename in os.listdir(gpx_path):
        file_path = os.path.join(gpx_path, filename)
        # 检查是否是文件
        if os.path.isfile(file_path) and file_path.endswith('.gpx'):
            track, _ = parse_xml(file_path)
            track.insert(0, 'id', range(1, 1 + len(track)))
            track.insert(1, 'order_no', range(1, 1 + len(track)))
            track.to_csv(output_path + "/" + filename[:-4] + ".csv", index=False)
