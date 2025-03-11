# Copyright 2020 - 2025 Ternaris
# SPDX-License-Identifier: Apache-2.0
"""Rosbag2 writer."""

# pyright: strict, reportUnreachable=false

from __future__ import annotations

import warnings
from io import StringIO
from pathlib import Path
from typing import TYPE_CHECKING

import zstandard
from ruamel.yaml import YAML

from rosbags.interfaces import (
    Connection,
    ConnectionExtRosbag2,
    MessageDefinition,
    MessageDefinitionFormat,
    Qos,
)
from rosbags.rosbag2.metadata import dump_qos_v8, dump_qos_v9, parse_qos
from rosbags.rosbag2.storage_mcap import McapWriter
from rosbags.rosbag2.storage_sqlite3 import Sqlite3Writer
from rosbags.typesys import Stores, get_typestore

from .enums import CompressionFormat, CompressionMode, StoragePlugin
from .errors import WriterError

if TYPE_CHECKING:
    import sys
    from collections.abc import Mapping, Sequence
    from types import TracebackType
    from typing import Literal, Protocol

    if sys.version_info >= (3, 11):
        from typing import Self
    else:
        from typing_extensions import Self

    from rosbags.typesys.store import Typestore

    from .metadata import Metadata

    class StorageWriter(Protocol):
        """Storage Writer Protocol."""

        path: Path

        def __init__(self, path: Path, compression: CompressionMode) -> None:
            """Initialize."""
            raise NotImplementedError

        def add_msgtype(self, connection: Connection) -> None:
            """Add a msgtypen."""
            raise NotImplementedError

        def add_connection(self, connection: Connection, offered_qos_profiles: str) -> None:
            """Add a connection."""
            raise NotImplementedError

        def write(self, connection: Connection, timestamp: int, data: bytes | memoryview) -> None:
            """Write message to rosbag2."""

        def close(self, version: int, metadata: str) -> None:
            """Close rosbag2 after writing."""


class DeprecationMeta(type):
    """Deprecation Meta Class."""

    def __getattr__(cls, name: str):  # type: ignore[no-untyped-def]  # noqa: ANN204
        """Deprecate enum members."""
        if name == 'CompressionMode':
            warnings.warn(
                'CompressionMode should be imported from rosbags.rosbag2.',
                category=DeprecationWarning,
                stacklevel=2,
            )
            return CompressionMode

        if name == 'CompressionFormat':
            warnings.warn(
                'CompressionFormat should be imported from rosbags.rosbag2.',
                category=DeprecationWarning,
                stacklevel=2,
            )
            return CompressionFormat
        msg = f'{cls.__name__!r} object has no attribute {name!r}'
        raise AttributeError(msg)


class Writer(metaclass=DeprecationMeta):
    """Rosbag2 writer.

    This class implements writing of rosbag2 files in version 8 or 9. It
    should be used as a contextmanager.

    """

    VERSION_LATEST: Literal[9] = 9

    STORAGE_PLUGINS: Mapping[StoragePlugin, type[StorageWriter]] = {
        StoragePlugin.SQLITE3: Sqlite3Writer,
        StoragePlugin.MCAP: McapWriter,
    }

    def __init__(
        self,
        path: Path | str,
        *,
        version: Literal[8, 9] | None = None,
        storage_plugin: StoragePlugin = StoragePlugin.SQLITE3,
    ) -> None:
        """Initialize writer.

        Args:
            path: Filesystem path to bag.
            version: Rosbag2 file format version.
            storage_plugin: Storage plugin to use.

        Raises:
            WriterError: Target path exists already, Writer can only create new rosbags.

        """
        path = Path(path)
        if path.exists():
            msg = f'{path} exists already, not overwriting.'
            raise WriterError(msg)

        if not version:
            warnings.warn(
                'Writer should be called with an explicit version number (8 or 9).',
                category=DeprecationWarning,
                stacklevel=2,
            )
            version = 8

        self.path = path
        self.metapath = path / 'metadata.yaml'
        self.version = version
        self.storage_plugin = storage_plugin

        self.compression_mode = CompressionMode.NONE
        self.compression_format = ''
        self.compressor: zstandard.ZstdCompressor | None = None

        self.storage: StorageWriter | None = None
        self.connections: list[Connection] = []
        self.counts: dict[int, int] = {}
        self.custom_data: dict[str, str] = {}
        self.added_types: set[str] = set()
        self.min_timestamp = 2**63 - 1
        self.max_timestamp = 0

    def set_compression(self, mode: CompressionMode, fmt: CompressionFormat) -> None:
        """Enable compression on bag.

        This function has to be called before opening.

        Args:
            mode: Compression mode to use, either 'file' or 'message'.
            fmt: Compressor to use, currently only 'zstd'.

        Raises:
            WriterError: Bag already open.

        """
        if self.storage:
            msg = f'Cannot set compression, bag {self.path} already open.'
            raise WriterError(msg)
        if mode == CompressionMode.NONE:
            return
        self.compression_mode = mode
        self.compression_format = fmt.name.lower()
        self.compressor = zstandard.ZstdCompressor()

    def set_custom_data(self, key: str, value: str) -> None:
        """Set key value pair in custom_data.

        Args:
            key: Key to set.
            value: Value to set.

        Raises:
            WriterError: If value has incorrect type.

        """
        if not isinstance(value, str):  # pyright:ignore[reportUnnecessaryIsInstance]
            msg = f'Cannot set non-string value {value!r} in custom_data.'
            raise WriterError(msg)
        self.custom_data[key] = value

    def open(self) -> None:
        """Open rosbag2 for writing.

        Create base directory and open database connection.

        """
        try:
            self.path.mkdir(mode=0o755, parents=True)
        except FileExistsError:
            msg = f'{self.path} exists already, not overwriting.'
            raise WriterError(msg) from None

        self.storage = self.STORAGE_PLUGINS[self.storage_plugin](self.path, self.compression_mode)

    def add_connection(
        self,
        topic: str,
        msgtype: str,
        *,
        typestore: Typestore | None = None,
        msgdef: str | None = None,
        rihs01: str | None = None,
        serialization_format: str = 'cdr',
        offered_qos_profiles: Sequence[Qos] | str = (),
    ) -> Connection:
        """Add a connection.

        This function can only be called after opening a bag.

        Args:
            topic: Topic name.
            msgtype: Message type.
            typestore: Typestore.
            msgdef: Message definition.
            rihs01: Message hash.
            serialization_format: Serialization format.
            offered_qos_profiles: QOS Profile.

        Returns:
            Connection object.

        Raises:
            WriterError: Bag not open or topic previously registered.

        """
        if not self.storage:
            msg = 'Bag was not opened.'
            raise WriterError(msg)

        if msgdef is None or rihs01 is None:
            if not typestore:
                warnings.warn(
                    'Writer.add_connection should be called with typestore or msgdef/rihs01 pair.',
                    category=DeprecationWarning,
                    stacklevel=2,
                )
                typestore = get_typestore(Stores.ROS2_FOXY)
            msgdef, _ = typestore.generate_msgdef(msgtype, ros_version=2)
            rihs01 = typestore.hash_rihs01(msgtype)
        assert msgdef is not None
        assert rihs01

        if isinstance(offered_qos_profiles, str):
            warnings.warn(
                'Writer.add_connection should be called with instantiated QoS profiles.',
                category=DeprecationWarning,
                stacklevel=2,
            )
            qos_profiles = parse_qos(offered_qos_profiles)
        else:
            qos_profiles = list(offered_qos_profiles)

        fmt = 'ros2idl' if msgdef.startswith('=' * 80 + '\nIDL: ') else 'ros2msg'

        fmtmap = {
            'ros2msg': MessageDefinitionFormat.MSG,
            'ros2idl': MessageDefinitionFormat.IDL,
        }

        connection = Connection(
            id=len(self.connections) + 1,
            topic=topic,
            msgtype=msgtype,
            msgdef=MessageDefinition(fmtmap[fmt], msgdef),
            digest=rihs01,
            msgcount=0,
            ext=ConnectionExtRosbag2(
                serialization_format=serialization_format,
                offered_qos_profiles=qos_profiles,
            ),
            owner=self,
        )
        for conn in self.connections:
            if (
                conn.topic == connection.topic
                and conn.msgtype == connection.msgtype
                and conn.ext == connection.ext
            ):
                msg = f'Connection can only be added once: {connection!r}.'
                raise WriterError(msg)

        self.connections.append(connection)
        self.counts[connection.id] = 0

        dump_qos = dump_qos_v9 if self.version >= 9 else dump_qos_v8
        dumped = dump_qos(qos_profiles)
        if not isinstance(dumped, str):
            stream = StringIO()
            yaml = YAML(typ='safe')
            yaml.default_flow_style = False
            yaml.dump(dumped, stream)  # pyright: ignore[reportUnknownMemberType]
            dumped = stream.getvalue().strip()

        if msgtype not in self.added_types:
            self.storage.add_msgtype(connection)
            self.added_types.add(msgtype)
        self.storage.add_connection(connection, dumped)
        return connection

    def write(self, connection: Connection, timestamp: int, data: bytes | memoryview) -> None:
        """Write message to rosbag2.

        Args:
            connection: Connection to write message to.
            timestamp: Message timestamp (ns).
            data: Serialized message data.

        Raises:
            WriterError: Bag not open or topic not registered.

        """
        if not self.storage:
            msg = 'Bag was not opened.'
            raise WriterError(msg)
        if connection not in self.connections:
            msg = f'Tried to write to unknown connection {connection!r}.'
            raise WriterError(msg)

        if self.compression_mode == CompressionMode.MESSAGE:
            assert self.compressor
            data = self.compressor.compress(data)

        self.storage.write(connection, timestamp, data)
        self.counts[connection.id] += 1
        self.min_timestamp = min(timestamp, self.min_timestamp)
        self.max_timestamp = max(timestamp, self.max_timestamp)

    def close(self) -> None:
        """Close rosbag2 after writing.

        Closes open database transactions and writes metadata.yaml.

        """
        assert self.storage

        compression_mode = (
            ''
            if self.compression_mode in {CompressionMode.NONE, CompressionMode.STORAGE}
            else self.compression_mode.name.lower()
        )

        compression_format = (
            ''
            if self.compression_mode in {CompressionMode.NONE, CompressionMode.STORAGE}
            else self.compression_format
        )

        path = self.storage.path
        dst = (
            path.with_suffix(f'{path.suffix}.{compression_format}')
            if self.compression_mode == CompressionMode.FILE
            else path
        )

        duration = max(0, self.max_timestamp - self.min_timestamp)
        start = self.min_timestamp
        count = sum(self.counts.values())

        dump_qos = dump_qos_v9 if self.version >= 9 else dump_qos_v8

        metadata: dict[str, Metadata] = {
            'rosbag2_bagfile_information': {
                'version': self.version,
                'storage_identifier': self.storage_plugin.name.lower(),
                'relative_file_paths': [dst.name],
                'duration': {'nanoseconds': duration},
                'starting_time': {'nanoseconds_since_epoch': start},
                'message_count': count,
                'topics_with_message_count': [
                    {
                        'topic_metadata': {
                            'name': x.topic,
                            'type': x.msgtype,
                            'serialization_format': x.ext.serialization_format,
                            'offered_qos_profiles': dump_qos(x.ext.offered_qos_profiles),
                            'type_description_hash': x.digest,
                        },
                        'message_count': self.counts[x.id],
                    }
                    for x in self.connections
                    if isinstance(x.ext, ConnectionExtRosbag2)
                ],
                'compression_format': compression_format,
                'compression_mode': compression_mode,
                'files': [
                    {
                        'path': dst.name,
                        'starting_time': {'nanoseconds_since_epoch': start},
                        'duration': {'nanoseconds': duration},
                        'message_count': count,
                    },
                ],
                'custom_data': self.custom_data or None,
                'ros_distro': 'rosbags',
            },
        }

        yaml = YAML(typ='safe')
        yaml.default_flow_style = False

        metastr = StringIO()
        yaml.dump(metadata, metastr)  # pyright: ignore[reportUnknownMemberType]
        self.metapath.write_text(metastr.getvalue(), 'utf8')

        metastr = StringIO()
        yaml.dump(metadata['rosbag2_bagfile_information'], metastr)  # pyright: ignore[reportUnknownMemberType]
        self.storage.close(self.version, metastr.getvalue().strip())
        self.storage = None

        if self.compression_mode == CompressionMode.FILE:
            assert self.compressor
            with path.open('rb') as infile, dst.open('wb') as outfile:
                _ = self.compressor.copy_stream(infile, outfile)
            path.unlink()

    def __enter__(self) -> Self:
        """Open rosbag2 when entering contextmanager."""
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> Literal[False]:
        """Close rosbag2 when exiting contextmanager."""
        self.close()
        return False

    def __getattribute__(self, name: str):  # type: ignore[no-untyped-def]  # noqa: ANN204
        """Deprecate enums."""
        if name == 'CompressionMode':
            return Writer.CompressionMode
        if name == 'CompressionFormat':
            return Writer.CompressionFormat
        return super().__getattribute__(name)
