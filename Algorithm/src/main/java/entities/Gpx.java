package entities;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.dataformat.xml.annotation.JacksonXmlElementWrapper;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Optional;

@JsonIgnoreProperties(
        ignoreUnknown = true
)
public class Gpx {
    @JacksonXmlElementWrapper(
            useWrapping = false
    )
    public List<Trk> trk = new ArrayList();
    @JacksonXmlElementWrapper(
            useWrapping = false
    )
    public List<Rte> rte = new ArrayList();

    public Gpx() {
    }

    @JsonIgnoreProperties(
            ignoreUnknown = true
    )
    public static class Trkpt {
        public double ele;
        public Date time;
        public double lat;
        public double lon;

        public Trkpt() {
        }
    }

    @JsonIgnoreProperties(
            ignoreUnknown = true
    )
    public static class Rtept {
        public double lat;
        public double lon;

        public Rtept() {
        }
    }

    @JsonIgnoreProperties(
            ignoreUnknown = true
    )
    public static class Trkseg {
        @JacksonXmlElementWrapper(
                useWrapping = false
        )
        public List<Trkpt> trkpt = new ArrayList();

        public Trkseg() {
        }
    }

    @JsonIgnoreProperties(
            ignoreUnknown = true
    )
    public static class Trk {
        @JacksonXmlElementWrapper(
                useWrapping = false
        )
        public List<Trkseg> trkseg = new ArrayList();
        public String name;

        public Trk() {
        }

        public Optional<Date> getStartTime() {
            return this.trkseg.stream().flatMap((trkseg) -> {
                return trkseg.trkpt.stream();
            }).findFirst().flatMap((trkpt) -> {
                return Optional.ofNullable(trkpt.time);
            });
        }
    }

    @JsonIgnoreProperties(
            ignoreUnknown = true
    )
    public static class Rte {
        @JacksonXmlElementWrapper(
                useWrapping = false
        )
        public List<Rtept> rtept = new ArrayList();

        public Rte() {
        }
    }
}
